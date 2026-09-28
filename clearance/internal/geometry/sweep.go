package geometry

import "math"

// Swept-query statuses. They are intentionally distinct from the single-frame
// status strings: a swept query answers "contact somewhere in the window" vs
// "safe throughout the window".
const (
	// SweptStatusContact means the parts touch (or already overlap) at a
	// reported time inside [0,1].
	SweptStatusContact = "contact"
	// SweptStatusSafe means they stay strictly separated over the whole
	// window; closest-approach information is reported instead.
	SweptStatusSafe = "safe"
)

// Additional kernel error codes used by the swept layer only.
const (
	// ErrNonFiniteVelocity: a translation velocity carries NaN / ±Inf.
	ErrNonFiniteVelocity = "NON_FINITE_VELOCITY"
	// ErrSweepNoConvergence: the conservative-advancement iteration used up
	// its step budget without bracketing contact or a closest approach.
	ErrSweepNoConvergence = "SWEEP_NO_CONVERGENCE"
)

// MaxSweepIterations bounds the number of single-frame Evaluate calls a swept
// query may consume. Like the GJK/EPA caps this is a hard guarantee: an
// exhausted budget is an error, never a silently fabricated "safe" answer.
const MaxSweepIterations = 256

// sweepMaxSteps is the live bound, indirected so tests can force exhaustion.
var sweepMaxSteps = MaxSweepIterations

const (
	// sweepContactGapRel: a positive gap no larger than this fraction of the
	// scene scale is regarded as geometric contact.
	sweepContactGapRel = 1e-9
	// sweepTimeTol brackets the first contact time (window is fixed [0,1],
	// so an absolute time tolerance is meaningful).
	sweepTimeTol = 1e-13
	// sweepClosestTimeTol brackets the closest-approach time.
	sweepClosestTimeTol = 1e-12
	// sweepRateRel: a closing rate (as a fraction of the relative speed) at
	// or below this is treated as "not approaching"; the clearance along a
	// straight relative path is convex in t, so once its derivative turns
	// non-negative it can never shrink again.
	sweepRateRel = 1e-12
)

// SweptInput describes two convex polygons, each with an initial pose offset
// (added to every vertex at t=0) and a constant translation velocity applied
// over the window [0,1]: part X occupies polygonX + offsetX + t*velocityX.
type SweptInput struct {
	A, B                 []Vec2
	OffsetA, OffsetB     Vec2
	VelocityA, VelocityB Vec2
}

// SweptResult is the swept-layer answer.
type SweptResult struct {
	// Status is SweptStatusContact or SweptStatusSafe.
	Status string `json:"status"`
	// Time is:
	//   contact -> the time of first impact in [0,1],
	//   safe    -> the time of closest approach in [0,1].
	Time float64 `json:"time"`
	// Distance is 0 for contact; for a safe sweep it is the minimum gap over
	// the window.
	Distance float64 `json:"distance"`
	// PenetrationDepth is non-zero only for the initially-penetrating case,
	// where it is copied verbatim from the t=0 single-frame result.
	PenetrationDepth float64 `json:"penetrationDepth"`
	// Normal is the unit contact/gap normal (from the witness on A toward
	// the witness on B), evaluated at Time.
	Normal Vec2 `json:"normal"`
	// PointA / PointB are the contact or closest points at Time, in world
	// coordinates (including the poses at that time). For a motion contact
	// they coincide.
	PointA Vec2 `json:"pointA"`
	PointB Vec2 `json:"pointB"`
	// RelativeVelocity is velocityB - velocityA, echoed for callers.
	RelativeVelocity Vec2 `json:"relativeVelocity"`
	// Iterations is the number of single-frame Evaluate calls consumed.
	Iterations int `json:"iterations"`
}

// sweptRun holds the mutable state of one swept query.
//
// Geometry is never reimplemented here: posedAt builds world-frame contours
// and eval delegates every distance/normal/witness question to the existing
// single-frame kernel.
type sweptRun struct {
	in      SweptInput
	w       Vec2    // relative velocity vB - vA
	mu      float64 // |w|
	tol     float64 // sweep-level contact gap tolerance
	kernTol float64 // single-frame kernel tolerance (distance-reporting floor)
	rateTol float64 // closing-rate "flat" threshold

	evals int
}

// Swept answers the continuous collision query for two convex polygons under
// constant translations over t in [0,1]. It never slices the window: time is
// advanced conservatively from the current gap and relative motion, and the
// first zero of the time-dependent clearance is bracketed and refined to the
// contact tolerance.
func Swept(in SweptInput) (*SweptResult, error) {
	// The same contour validation the static entry point performs, with the
	// same "polygon A/B" attribution.
	if _, err := NewPolygon(in.A); err != nil {
		return nil, namedError("A", err.(*KernelError))
	}
	if _, err := NewPolygon(in.B); err != nil {
		return nil, namedError("B", err.(*KernelError))
	}
	if err := finiteOffset(in.OffsetA, "A"); err != nil {
		return nil, err
	}
	if err := finiteOffset(in.OffsetB, "B"); err != nil {
		return nil, err
	}
	if err := finiteVelocity(in.VelocityA, "A"); err != nil {
		return nil, err
	}
	if err := finiteVelocity(in.VelocityB, "B"); err != nil {
		return nil, err
	}

	w := in.VelocityB.Sub(in.VelocityA)
	scale := max(posedScale(in.A, in.OffsetA), posedScale(in.B, in.OffsetB), w.Len())

	sw := &sweptRun{
		in:      in,
		w:       w,
		mu:      w.Len(),
		tol:     sweepContactGapRel * max(1.0, scale),
		kernTol: eps * max(1.0, scale),
		rateTol: sweepRateRel * w.Len(),
	}

	r0, err := sw.eval(0)
	if err != nil {
		return nil, err
	}

	// Zero relative velocity (common motion or both at rest): the relative
	// geometry is frozen, so the answer must be exactly the static t=0
	// conclusion.
	if w.Len2() == 0 {
		return sw.staticEquivalent(r0), nil
	}
	// Already overlapping at the start: first contact time is 0 and the
	// contact information is the static penetration result.
	if r0.Status == StatusPenetrated {
		return sw.contactAtZero(r0), nil
	}

	t := 0.0
	res := r0
	// Last frame known to be approaching; used to bracket interior minima.
	appT, appR := 0.0, r0

	for {
		d := res.Distance
		if d <= sw.tol {
			// The conservative advance has walked into the geometric
			// contact band without the single-frame predicate flipping
			// (a tangent "grazing" path can do this). Refine the true
			// first zero rather than reporting the band-edge time.
			return sw.refineImpact(t, res)
		}
		closing := -w.Dot(res.Normal) // >0 while the witness gap is closing

		if closing <= sw.rateTol {
			// The clearance along a straight relative path is a convex
			// function of t, so once its derivative (-closing) becomes
			// non-negative the gap cannot shrink again. If an earlier frame
			// was still approaching, the minimum lies between the two.
			if appT < t {
				return sw.refineClosest(appT, t, appR, res)
			}
			return sw.safeResult(res, t), nil
		}

		// Conservative advance: the distance between two convex sets under a
		// relative translation u is 1-Lipschitz in u, hence over a step dt
		// the gap can decrease by at most |w|*dt. Advancing by d/|w| cannot
		// pass a contact instant.
		step := d / sw.mu
		remaining := 1.0 - t
		if step >= remaining {
			r1, err := sw.eval(1)
			if err != nil {
				return nil, err
			}
			return sw.atWindowEnd(t, res, r1)
		}

		tNext := t + step
		if tNext <= t {
			// Defensive: zero floating progress without reaching the
			// contact tolerance must not spin forever.
			return nil, sw.noConvergence()
		}
		rNext, err := sw.eval(tNext)
		if err != nil {
			return nil, err
		}
		if rNext.Status == StatusPenetrated {
			// Contact was entered between two consecutive conservative
			// samples, so the true first impact is bracketed.
			return sw.refineContact(t, tNext, res, false)
		}
		// Still separated: the current frame approaches, remember it before
		// advancing.
		appT, appR = t, res
		t, res = tNext, rNext
	}
}

// atWindowEnd resolves the query when the conservative advance reaches (or
// would pass) t=1. lo/rLo is a separated frame strictly at or before the
// window interior; rHi is the frame at t=1.
func (sw *sweptRun) atWindowEnd(lo float64, rLo, rHi *Result) (*SweptResult, error) {
	if rHi.Status == StatusPenetrated {
		// Either an impact earlier in the window or an exact end-instant
		// touch: bracket the entry against the last separated frame.
		return sw.refineContact(lo, 1, rLo, true)
	}
	if rHi.Distance <= sw.tol {
		// Exactly tangent at t=1 (within the geometric contact tolerance):
		// refine the true zero, rounding a genuine end touch onto 1.
		out, err := sw.refineImpact(1, rHi)
		if err != nil {
			return nil, err
		}
		if math.Abs(1-out.Time) <= 1e-11 {
			out.Time = 1
		}
		return out, nil
	}
	closingEnd := -sw.w.Dot(rHi.Normal)
	if closingEnd <= sw.rateTol {
		// Already receding (or flat) at the end: the minimum is interior.
		if -sw.w.Dot(rLo.Normal) <= sw.rateTol {
			// It was already not approaching at the last interior frame:
			// the minimum is there.
			if rLo.Distance <= rHi.Distance {
				return sw.safeResult(rLo, lo), nil
			}
			return sw.safeResult(rHi, 1), nil
		}
		return sw.refineClosest(lo, 1, rLo, rHi)
	}
	// Still approaching when the window closes: t=1 is the closest instant.
	return sw.safeResult(rHi, 1), nil
}

// refineImpact locates the first zero of the clearance given a frame t0
// whose (still positive) gap is already inside the geometric contact band.
// It probes one Lipschitz step d/|w| ahead, which provably lands at or beyond
// the first zero (the gap can shrink by at most |w|*dt), then hands the
// bracket to bisectRoot (guarded Newton + bisection + a bias-cancelling
// secant). If the probe is still clearly separated — the closest feature
// rotated away before contact — conservative advancement resumes from the
// probe instead of forcing an unbracketed root.
func (sw *sweptRun) refineImpact(t0 float64, r0 *Result) (*SweptResult, error) {
	// Common entry: r0 is a strictly separated frame whose gap is already
	// inside the geometric contact band. Probe one Lipschitz step ahead;
	// because the gap can shrink by at most |w|*dt, that probe lands at or
	// beyond the first zero, yielding a contacted frame that brackets the
	// root with t0 on the strictly separated side.
	step := r0.Distance / sw.mu
	tHi := t0 + step
	if tHi > 1 {
		tHi = 1
	}
	rHi, err := sw.eval(tHi)
	if err != nil {
		return nil, err
	}
	gHi := rHi.Distance
	if rHi.Status == StatusPenetrated {
		gHi = -rHi.PenetrationDepth
	}
	if gHi > sw.tol {
		// The probe did not reach contact (the witness normal rotated away).
		// Continue conservative advancement from the probe instead of
		// forcing a root that is not bracketed.
		return sw.advanceFrom(tHi, rHi)
	}
	tc, sepT, rSep, err := sw.bisectRoot(t0, tHi, r0, rHi)
	if err != nil {
		return nil, err
	}
	if math.Abs(1-tc) <= 1e-11 {
		tc = 1
	}
	return sw.contactSnap(rSep, sepT, tc), nil
}

// advanceFrom continues the main conservative-advancement loop starting from
// an already evaluated frame; used when refineImpact's probe does not reach
// contact because the closest feature rotated.
func (sw *sweptRun) advanceFrom(t0 float64, r0 *Result) (*SweptResult, error) {
	t, res := t0, r0
	appT, appR := t0, r0
	for {
		d := res.Distance
		if d <= sw.tol {
			return sw.refineImpact(t, res)
		}
		closing := -sw.w.Dot(res.Normal)
		if closing <= sw.rateTol {
			if appT < t {
				return sw.refineClosest(appT, t, appR, res)
			}
			return sw.safeResult(res, t), nil
		}
		step := d / sw.mu
		remaining := 1.0 - t
		if step >= remaining {
			r1, err := sw.eval(1)
			if err != nil {
				return nil, err
			}
			return sw.atWindowEnd(t, res, r1)
		}
		tNext := t + step
		if tNext <= t {
			return nil, sw.noConvergence()
		}
		rNext, err := sw.eval(tNext)
		if err != nil {
			return nil, err
		}
		if rNext.Status == StatusPenetrated {
			return sw.refineContact(t, tNext, res, false)
		}
		appT, appR = t, res
		t, res = tNext, rNext
	}
}

// bisectRoot narrows a bracket (lo strictly separated, hi at/past contact)
// to the first zero of the clearance, and returns the impact time together
// with the last strictly separated frame (used for the contact normal and
// witness points).
//
// Refinement has three ingredients, applied in order:
//
//  1. Guarded Newton steps t' = t + d/closing (closing = -(vB-vA)·n) from
//     separated frames. Convexity of the clearance along a straight path
//     guarantees t' never passes the first zero, so each step is a sound,
//     rapidly tightening lower bound on the impact time.
//  2. Bisection on the single-frame contacted/separated predicate once Newton
//     can no longer advance (or from the outset for a tangent "grazing" cusp
//     whose closing rate tends to zero).
//  3. A two-frame secant extrapolation of the true zero from the two tightest
//     strictly separated frames. The single-frame kernel reports a small,
//     nearly constant positive bias on the gap near contact (it rounds gaps
//     below its own epsilon onto the contact branch); a one-frame Newton
//     polish inherits that bias in time. The secant uses the gap SLOPE
//     between two separated frames, the constant bias cancels, and the impact
//     time matches an analytic root to machine precision (~1e-13). For the
//     grazing cusp the secant slope is ill-conditioned (rate -> 0) and the
//     bracket midpoint is returned instead.
func (sw *sweptRun) bisectRoot(lo, hi float64, rLo, rHi *Result) (tc, sepT float64, sep *Result, err error) {
	// The two tightest (largest-time) strictly separated frames whose gaps
	// still sit above the single-frame distance floor, kept ordered as
	// t0 < t1 with gaps d0, d1. Frames below that floor report a saturated,
	// quantized gap AND a quantized (axis-snapped) normal, so they are used
	// neither for the slope fit nor for the contact normal/witnesses.
	var t0, d0, t1, d1 float64
	var r1 *Result
	have := 0
	gapFloor := 8 * sw.kernTol
	note := func(t float64, r *Result) {
		if r.Status != StatusSeparated || r.Distance < gapFloor {
			return
		}
		switch {
		case have == 0:
			t1, d1, r1, have = t, r.Distance, r, 1
		case have == 1:
			if t >= t1 {
				t0, d0, t1, d1, r1 = t1, d1, t, r.Distance, r
			} else {
				t0, d0 = t, r.Distance
			}
			have = 2
		default:
			if t > t1 {
				t0, d0, t1, d1, r1 = t1, d1, t, r.Distance, r
			} else if t > t0 {
				t0, d0 = t, r.Distance
			}
		}
	}
	note(lo, rLo)

	// Phase 1: iterated guarded Newton from the separated side.
	t, r := lo, rLo
	for i := 0; i < sweepMaxSteps; i++ {
		note(t, r)
		if hi-lo <= sweepTimeTol {
			break
		}
		if r.Status == StatusPenetrated || r.Distance <= sw.tol {
			break // entered the kernel contact band: bisect the band
		}
		closing := -sw.w.Dot(r.Normal)
		if closing <= sw.rateTol {
			break // grazing cusp: bisect
		}
		tNext := t + r.Distance/closing
		if tNext <= t {
			break
		}
		if tNext >= hi {
			tNext = (t + hi) * 0.5 // keep the estimate inside the bracket
		}
		rn, err := sw.eval(tNext)
		if err != nil {
			return 0, 0, nil, err
		}
		if rn.Status == StatusPenetrated || rn.Distance <= sw.tol {
			hi, rHi = tNext, rn
			break
		}
		lo, rLo = tNext, rn
		t, r = tNext, rn
	}

	// Phase 2: bisection on the KERNEL's contacted/separated predicate (not
	// the sweep-level geometric band sw.tol): only the kernel knows the true
	// sign at this scale. Strictly separated frames down to ~1e-10 are
	// recorded so the phase-3 secant can fit the gap slope through the two
	// tightest of them; using the looser sweep band here would discard
	// precisely the frames needed to cancel the gap bias.
	for hi-lo > sweepTimeTol {
		mid := (lo + hi) * 0.5
		rm, err := sw.eval(mid)
		if err != nil {
			return 0, 0, nil, err
		}
		if rm.Status == StatusPenetrated {
			hi, rHi = mid, rm
		} else {
			note(mid, rm)
			lo, rLo = mid, rm
		}
	}

	// Phase 3: bias-cancelling secant from the two tightest separated frames.
	// The secant is extrapolated beyond the tightest separated frame, so its
	// zero can coincide with the geometric root even though the bisection
	// bracket's upper edge is the kernel's contact-band edge, slightly BEFORE
	// the root. Clamp the estimate to the time window rather than to that
	// bracket: for a convex clearance the secant chord lies on/above the
	// curve, so its zero overshoots the true root by at most the curvature
	// over one sub-step (negligible at the chosen bracket width).
	tc = (lo + hi) * 0.5
	if have == 2 && t1 > t0 {
		slope := (d1 - d0) / (t1 - t0) // < 0 while approaching
		if slope < -sw.rateTol {
			if est := t1 - d1/slope; est >= lo && est <= 1 {
				tc = est
			}
		}
	}
	// tc: refined impact time. The contact normal and witnesses come from
	// the tightest well-resolved separated frame r1 at t1 (frames below the
	// distance floor carry a quantized normal); the caller advances those
	// witnesses forward to tc. Fall back to the tightest frame if the search
	// never left the floor band.
	sepFrame := rLo
	sepTime := lo
	if have >= 1 && r1 != nil {
		sepFrame, sepTime = r1, t1
	}
	return tc, sepTime, sepFrame, nil
}

// refineContact brackets the first contact time when a penetrated frame was
// hit during conservative advancement. Invariants:
//   - rLo is a separated frame at lo,
//   - the frame at hi is penetrated,
//   - the true entry time lies in (lo, hi].
func (sw *sweptRun) refineContact(lo, hi float64, rLo *Result, endTouch bool) (*SweptResult, error) {
	rHi, err := sw.eval(hi)
	if err != nil {
		return nil, err
	}
	tc, sepT, rSep, err := sw.bisectRoot(lo, hi, rLo, rHi)
	if err != nil {
		return nil, err
	}
	// An exact end-instant touch rounds onto 1 rather than 1-epsilon.
	if endTouch && math.Abs(1-tc) <= 1e-11 {
		tc = 1
	}
	return sw.contactSnap(rSep, sepT, tc), nil
}

// refineClosest brackets the time of minimum gap between an approaching
// frame (lo) and a non-approaching frame (hi), both separated. Bisection
// follows the sign of the gap derivative; the smallest frame sampled is the
// reported closest approach. A penetrated midpoint (defensive: only possible
// near a clamped window end) reroutes to contact refinement.
func (sw *sweptRun) refineClosest(lo, hi float64, rLo, rHi *Result) (*SweptResult, error) {
	bestT, bestR := lo, rLo
	if rHi.Distance < bestR.Distance {
		bestT, bestR = hi, rHi
	}
	for hi-lo > sweepClosestTimeTol {
		mid := (lo + hi) * 0.5
		rm, err := sw.eval(mid)
		if err != nil {
			return nil, err
		}
		if rm.Status == StatusPenetrated {
			return sw.refineContact(lo, mid, rLo, mid == 1.0)
		}
		if rm.Distance < bestR.Distance {
			bestT, bestR = mid, rm
		}
		if -sw.w.Dot(rm.Normal) > sw.rateTol {
			lo, rLo = mid, rm
		} else {
			hi = mid
		}
	}
	return sw.safeResult(bestR, bestT), nil
}

// eval returns the single-frame kernel result at time t, counting the call
// against the iteration budget.
func (sw *sweptRun) eval(t float64) (*Result, error) {
	sw.evals++
	if sw.evals > sweepMaxSteps {
		return nil, sw.noConvergence()
	}
	return Evaluate(posedAt(sw.in.A, sw.in.OffsetA, sw.in.VelocityA, t),
		posedAt(sw.in.B, sw.in.OffsetB, sw.in.VelocityB, t))
}

func (sw *sweptRun) noConvergence() *KernelError {
	return &KernelError{Code: ErrSweepNoConvergence,
		Message: "swept conservative-advancement did not converge within the step budget"}
}

// staticEquivalent maps a single-frame t=0 result one-to-one onto the swept
// response (zero relative velocity branch).
func (sw *sweptRun) staticEquivalent(r *Result) *SweptResult {
	out := &SweptResult{
		Status:           SweptStatusSafe,
		Time:             0,
		Distance:         r.Distance,
		PenetrationDepth: 0,
		Normal:           r.Normal,
		PointA:           r.PointA,
		PointB:           r.PointB,
		RelativeVelocity: sw.w,
		Iterations:       sw.evals,
	}
	if r.Status == StatusPenetrated {
		out.Status = SweptStatusContact
		out.Distance = 0
		out.PenetrationDepth = r.PenetrationDepth
	}
	return out
}

// contactAtZero reproduces the initial-frame penetration conclusion with a
// first contact time of 0.
func (sw *sweptRun) contactAtZero(r *Result) *SweptResult {
	return &SweptResult{
		Status:           SweptStatusContact,
		Time:             0,
		Distance:         0,
		PenetrationDepth: r.PenetrationDepth,
		Normal:           r.Normal,
		PointA:           r.PointA,
		PointB:           r.PointB,
		RelativeVelocity: sw.w,
		Iterations:       sw.evals,
	}
}

// contactSnap builds a contact response from the last strictly separated
// frame r at time sepT. Its two witnesses are advanced by each part's own
// velocity over dt = tc-sepT so they are reported at the impact time tc; the
// relative advance is -w*dt, closing exactly the residual gap along the
// normal (to the accuracy of the refined root). The two near-coincident
// witnesses are then snapped to their midpoint, yielding one contact point.
func (sw *sweptRun) contactSnap(r *Result, sepT, tc float64) *SweptResult {
	dt := tc - sepT
	pa := r.PointA.Add(sw.in.VelocityA.Scale(dt))
	pb := r.PointB.Add(sw.in.VelocityB.Scale(dt))
	c := pa.Add(pb.Sub(pa).Scale(0.5)).cleanZeros()
	return &SweptResult{
		Status:           SweptStatusContact,
		Time:             tc,
		Distance:         0,
		PenetrationDepth: 0,
		Normal:           r.Normal,
		PointA:           c,
		PointB:           c,
		RelativeVelocity: sw.w,
		Iterations:       sw.evals,
	}
}

// safeResult builds a "safe over the whole window" response.
func (sw *sweptRun) safeResult(r *Result, t float64) *SweptResult {
	return &SweptResult{
		Status:           SweptStatusSafe,
		Time:             t,
		Distance:         r.Distance,
		PenetrationDepth: 0,
		Normal:           r.Normal,
		PointA:           r.PointA,
		PointB:           r.PointB,
		RelativeVelocity: sw.w,
		Iterations:       sw.evals,
	}
}

// posedAt returns the world-frame contour base + off + t*v.
func posedAt(base []Vec2, off, v Vec2, t float64) []Vec2 {
	d := off.Add(v.Scale(t))
	out := make([]Vec2, len(base))
	for i, p := range base {
		out[i] = p.Add(d)
	}
	return out
}

// posedScale bounds the coordinate magnitude of the posed contour, used to
// scale the geometric tolerances the same way epsFor does in the kernel.
func posedScale(pts []Vec2, off Vec2) float64 {
	m := 1.0
	for _, p := range pts {
		q := p.Add(off)
		m = max(m, math.Abs(q.X), math.Abs(q.Y))
	}
	return m
}

func finiteOffset(v Vec2, which string) error {
	if !isFinite(v.X) || !isFinite(v.Y) {
		return &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "offset " + which + ": offset coordinates must be finite numbers"}
	}
	return nil
}

func finiteVelocity(v Vec2, which string) error {
	if !isFinite(v.X) || !isFinite(v.Y) {
		return &KernelError{Code: ErrNonFiniteVelocity,
			Message: "velocity " + which + ": velocity components must be finite numbers"}
	}
	return nil
}
