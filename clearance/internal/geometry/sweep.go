package geometry

import "math"

// Verdict values reported by the sweep query.
const (
	// VerdictContact: the parts come into contact at some time in [0,1].
	VerdictContact = "contact"
	// VerdictSafe: the parts stay strictly separated over the whole window.
	VerdictSafe = "safe"
)

// SweepResult is the kernel answer for a swept (time-dependent) query: two
// convex polygons translating with constant velocities over t in [0,1].
type SweepResult struct {
	// Verdict is VerdictContact or VerdictSafe.
	Verdict string `json:"verdict"`
	// TimeOfImpact is the first contact time in [0,1] (contact verdict only;
	// 0 when the parts already overlap at the start).
	TimeOfImpact float64 `json:"timeOfImpact"`
	// TimeOfClosestApproach is the time of minimum clearance (safe verdict
	// only; 0 for receding or zero relative motion).
	TimeOfClosestApproach float64 `json:"timeOfClosestApproach"`
	// MinDistance is the clearance at the closest approach (safe only).
	MinDistance float64 `json:"minDistance"`
	// PenetrationDepth is the overlap at the impact time: ~0 for an impact
	// during the window, the true depth when the parts start overlapped.
	PenetrationDepth float64 `json:"penetrationDepth"`
	// Normal is the contact/gap normal at the reported instant, with the
	// same convention as the static query (from part A toward part B).
	Normal Vec2 `json:"normal"`
	// PointA / PointB are the contact (or closest) points on the two parts
	// at the reported instant, in world coordinates at that time.
	PointA Vec2 `json:"pointA"`
	PointB Vec2 `json:"pointB"`
	// Iterations is the number of static frame evaluations consumed.
	Iterations int `json:"iterations"`
}

const (
	// sweepContactTol is the relative clearance below which the parts count
	// as being in contact (same scale as the static kernel's eps).
	sweepContactTol = 1e-10
	// sweepTimeTol is the absolute tolerance on the closest-approach time
	// bracket produced by the derivative bisection.
	sweepTimeTol = 1e-12
	// MaxSweepSteps bounds the root-finding (conservative advancement)
	// iterations; MaxSweepBisectSteps bounds the closest-approach bisection.
	// Exhausting either is reported as SWEEP_NO_CONVERGENCE, never silently
	// as "safe".
	MaxSweepSteps       = 64
	MaxSweepBisectSteps = 64
)

// The live bounds, indirected so tests can force exhaustion.
var sweepMaxSteps = MaxSweepSteps
var sweepBisectSteps = MaxSweepBisectSteps

// sweeper carries one sweep query: the two contours at t=0, their constant
// velocities, and a counter of static frame evaluations.
type sweeper struct {
	a, b   []Vec2
	va, vb Vec2
	// vrel is the relative velocity vB - vA. Only the relative motion
	// matters for the gap function; adding a common velocity to both parts
	// (or a common offset to both contours) leaves the verdict unchanged.
	vrel  Vec2
	tol   float64
	evals int
}

// Sweep determines whether two convex polygons, each translating with its
// own constant velocity over the time window t in [0,1], come into contact,
// and if so when they first touch.
//
// The clearance g(t) of two convex bodies under relative translation is the
// distance from the linearly moving point t·vrel to the fixed Minkowski
// difference A⊖B — a convex function of t. Where g(t) > 0 its exact
// derivative is vrel·n(t), with n(t) the gap normal reported by the static
// kernel. The static single-frame evaluation is reused as a subroutine at
// every step; no geometry is re-implemented here.
//
// Root finding uses conservative advancement: from t with gap g and closing
// rate -vrel·n > 0, advancing by g/(-vrel·n) follows the tangent of the
// convex gap function, which is a global lower bound — the iterates
// increase monotonically and never step past the true first contact time.
// No time sampling is involved: a contact that flashes by inside the window
// is still converged to.
func Sweep(aPts, bPts []Vec2, velA, velB Vec2) (*SweepResult, error) {
	if !isFinite(velA.X) || !isFinite(velA.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "velocity of part A must have finite components"}
	}
	if !isFinite(velB.X) || !isFinite(velB.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "velocity of part B must have finite components"}
	}

	s := &sweeper{
		a:    aPts,
		b:    bPts,
		va:   velA,
		vb:   velB,
		vrel: velB.Sub(velA),
		tol:  sweepContactTol * sweepScale(aPts, bPts),
	}

	// Frame t=0; this also validates both contours with exactly the same
	// checks and error codes as the static query.
	r0, err := s.at(0)
	if err != nil {
		return nil, err
	}

	// Already overlapping at the start: the first contact time is 0 by
	// definition and the contact information is the static penetration
	// answer of the initial pose.
	if r0.Status == StatusPenetrated {
		return s.contact(0, r0), nil
	}

	// Zero relative motion: the gap is constant over the window, so the
	// sweep conclusion is exactly the static evaluation of the initial
	// pose (same gap, same normal), with the closest approach at t=0.
	if s.vrel.Len2() == 0 {
		return s.safeResult(0, r0), nil
	}

	// Not closing at t=0 (receding or exactly parallel motion): the convex
	// gap function is then non-decreasing on the whole window, so the
	// closest approach is the initial frame and there is no impact.
	if -s.vrel.Dot(r0.Normal) <= 0 {
		return s.safeResult(0, r0), nil
	}

	// Conservative advancement. Loop invariant: the evaluation r at the
	// current time t is separated and still closing (rate > 0).
	t, tPrev := 0.0, 0.0
	r := r0
	for step := 1; step <= sweepMaxSteps; step++ {
		gap := r.Distance
		rate := -s.vrel.Dot(r.Normal)
		if gap <= s.tol {
			// Within contact tolerance: polish the impact time with one
			// final tangent step. The contact normal is the gap normal of
			// this separated frame — the tangent normal at the touch.
			tC := t
			if rate > 0 {
				tC = math.Min(1, t+gap/rate)
			}
			return s.contactAt(tC, r.Normal, r), nil
		}
		if rate <= 0 {
			// Unreachable under the loop invariant; kept as a guard.
			return s.closestIn(tPrev, t, r)
		}
		tNext := t + gap/rate
		if tNext > 1 {
			// The root lies beyond the window: no impact. The minimum is
			// at t=1 or at an interior point of [t,1].
			return s.closestIn(t, 1, nil)
		}
		tPrev, t = t, tNext
		rPrev := r
		r, err = s.at(t)
		if err != nil {
			return nil, err
		}
		if r.Status == StatusPenetrated {
			// The clearance closed to below the kernel's own tolerance:
			// contact at the current time (a tangent step never overshoots
			// the first root, so this is the first contact). The contact
			// normal is the gap normal of the last separated frame — the
			// limit normal at the touch, which is well defined even for a
			// vertex-vertex touch where the penetration branch's normal
			// would be ambiguous. The contact points come from the touch
			// frame itself, where the two witness points coincide.
			return s.contactAt(t, rPrev.Normal, r), nil
		}
		if -s.vrel.Dot(r.Normal) <= 0 {
			// The gap started growing again without ever reaching the
			// contact tolerance: no impact; the closest approach lies
			// inside [tPrev, t].
			return s.closestIn(tPrev, t, r)
		}
	}
	return nil, &KernelError{Code: ErrSweepNoConvergence,
		Message: "sweep root finding did not converge within the step budget"}
}

// closestIn builds the safe verdict for a query whose gap is known to stay
// positive over [lo, hi] ⊆ [0,1], with the gap still closing at lo. rHi is
// the static evaluation at hi, or nil if hi has not been evaluated yet.
//
// The gap derivative vrel·n(t) is continuous and non-decreasing (the gap is
// convex in t), so the closest approach is found by bisecting its sign
// change — again driven solely by static kernel evaluations.
func (s *sweeper) closestIn(lo, hi float64, rHi *Result) (*SweepResult, error) {
	if rHi == nil {
		var err error
		rHi, err = s.at(hi)
		if err != nil {
			return nil, err
		}
	}
	if rHi.Status == StatusPenetrated {
		// Boundary case: the gap at hi is below the kernel tolerance,
		// which counts as a touch at hi.
		return s.contact(hi, rHi), nil
	}
	if hi <= lo || -s.vrel.Dot(rHi.Normal) >= 0 {
		// Still closing at hi (or a degenerate bracket): the minimum over
		// the bracket is at hi.
		return s.safeResult(hi, rHi), nil
	}

	loB, hiB := lo, hi
	for i := 0; i < sweepBisectSteps; i++ {
		if hiB-loB <= sweepTimeTol {
			break
		}
		mid := 0.5 * (loB + hiB)
		rm, err := s.at(mid)
		if err != nil {
			return nil, err
		}
		if rm.Status == StatusPenetrated {
			return s.contact(mid, rm), nil
		}
		if -s.vrel.Dot(rm.Normal) > 0 {
			loB = mid // still closing: the minimum is to the right
		} else {
			hiB = mid
		}
	}
	if hiB-loB > sweepTimeTol {
		return nil, &KernelError{Code: ErrSweepNoConvergence,
			Message: "sweep closest-approach bisection did not converge within the step budget"}
	}
	tMin := 0.5 * (loB + hiB)
	rMin, err := s.at(tMin)
	if err != nil {
		return nil, err
	}
	if rMin.Status == StatusPenetrated {
		return s.contact(tMin, rMin), nil
	}
	return s.safeResult(tMin, rMin), nil
}

// at evaluates the static kernel on the two contours advanced to time t.
func (s *sweeper) at(t float64) (*Result, error) {
	s.evals++
	return Evaluate(advance(s.a, s.va, t), advance(s.b, s.vb, t))
}

// contact builds the contact verdict at time t from the static evaluation
// at that instant, taking the normal from that same frame. Used when the
// parts already overlap at t=0 (the static penetration answer is the
// required contact information) and for the fuzzy boundary flips inside
// closestIn, where the gap fell below the kernel tolerance.
func (s *sweeper) contact(t float64, r *Result) *SweepResult {
	return s.contactAt(t, r.Normal, r)
}

// contactAt builds the contact verdict for an impact at time t: the contact
// normal n is supplied by the caller (for impacts inside the window it is
// the gap normal of the last separated frame — the tangent normal at the
// touch), while the contact points and depth come from the static
// evaluation r at the touch instant.
func (s *sweeper) contactAt(t float64, n Vec2, r *Result) *SweepResult {
	return &SweepResult{
		Verdict:          VerdictContact,
		TimeOfImpact:     t,
		PenetrationDepth: r.PenetrationDepth,
		Normal:           n,
		PointA:           r.PointA,
		PointB:           r.PointB,
		Iterations:       s.evals,
	}
}

// safeResult builds the safe verdict with closest approach at tMin.
func (s *sweeper) safeResult(tMin float64, r *Result) *SweepResult {
	return &SweepResult{
		Verdict:               VerdictSafe,
		TimeOfClosestApproach: tMin,
		MinDistance:           r.Distance,
		Normal:                r.Normal,
		PointA:                r.PointA,
		PointB:                r.PointB,
		Iterations:            s.evals,
	}
}

// advance translates a contour by v*t.
func advance(pts []Vec2, v Vec2, t float64) []Vec2 {
	d := v.Scale(t)
	out := make([]Vec2, len(pts))
	for i, p := range pts {
		out[i] = p.Add(d)
	}
	return out
}

// sweepScale is the characteristic length used to scale the contact
// tolerance: the largest coordinate magnitude of the two initial contours,
// floored at 1 (mirroring the static kernel's tolerance scaling).
func sweepScale(a, b []Vec2) float64 {
	m := 1.0
	for _, p := range a {
		m = math.Max(m, math.Max(math.Abs(p.X), math.Abs(p.Y)))
	}
	for _, p := range b {
		m = math.Max(m, math.Max(math.Abs(p.X), math.Abs(p.Y)))
	}
	return m
}
