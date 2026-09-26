package geometry

import "math"

// Status values reported by the sweep (continuous collision) query.
const (
	// StatusImpact: the two parts touch at SweepResult.Time within [0,1].
	StatusImpact = "impact"
	// StatusClear: the two parts stay separated for the whole window [0,1].
	StatusClear = "clear"
)

// MaxSweepIterations bounds the conservative-advancement steps of the sweep
// solver. A genuine run lands on the contact time in a handful of steps —
// the advancement is Newton-like on the convex gap function and is exact
// once the closest feature pair stops changing. Exhausting the budget is
// reported as SWEEP_NO_CONVERGENCE, never as a silent "safe" answer and
// never as an infinite loop.
const MaxSweepIterations = 128

// sweepMaxSteps is the live bound, indirected so tests can force exhaustion.
var sweepMaxSteps = MaxSweepIterations

// sweepBisectSteps is the fixed iteration count of the bracketing
// refinements (defensive impact bisection and closest-approach search).
// It is a hard-coded constant, so these refinements always terminate.
const sweepBisectSteps = 60

// sweepContactRel is the relative contact tolerance: a frame whose gap (or
// penetration depth) is within this fraction of the geometry scale counts
// as touching. It sits one decade above the static kernel's internal
// routing tolerance so boundary frames are classified consistently.
const sweepContactRel = 1e-9

// SweepResult is the answer of the continuous (swept) query over t in [0,1].
type SweepResult struct {
	// Status is StatusImpact or StatusClear.
	Status string `json:"status"`
	// Time is the first-contact time (impact) or the closest-approach time
	// (clear), always within [0,1].
	Time float64 `json:"time"`
	// Distance is 0 on impact and the minimum gap over the window when clear.
	Distance float64 `json:"distance"`
	// PenetrationDepth is positive only when the parts already overlap at
	// t=0: the impact time is then 0 by definition and the contact
	// information is the static penetration verdict at the initial pose.
	PenetrationDepth float64 `json:"penetrationDepth"`
	// Normal is the contact normal at Time, pointing from part A toward
	// part B (same convention as the static query).
	Normal Vec2 `json:"normal"`
	// PointA / PointB are the contact (impact) or closest (clear) points on
	// the two parts at Time, in world coordinates of that instant.
	PointA Vec2 `json:"pointA"`
	PointB Vec2 `json:"pointB"`
	// Iterations is the number of conservative-advancement steps consumed.
	Iterations int `json:"iterations"`
}

// Sweep answers the continuous query: two convex polygons start at the given
// poses at t=0 and translate with the constant velocities va, vb over the
// window t in [0,1]. It reports whether the parts touch within the window;
// on impact it gives the first-contact time and the contact normal/points at
// that instant, on clearance the time of closest approach with the minimum
// gap and the gap normal.
//
// The single-frame kernel is reused as a subroutine — no geometry is
// re-implemented here and no fixed time-grid sampling is used. The gap g(t)
// between the linearly moving parts is a convex function of t, and the
// static query at time t supplies both g(t) and the gap normal n(t). With
// the relative velocity vrel = vb-va, the closing rate c = -vrel·n gives
// the supporting-halfspace bound
//
//	g(t+s) >= g(t) - c·s   for all s >= 0,
//
// so advancing by dt = g/c is conservative (it can never skip a contact)
// and lands exactly on the contact time once the closest feature pair stops
// changing. Contacts that flash by inside any coarse sample interval are
// still caught, and the reported time converges to the true first-contact
// time rather than to a sampling grid.
func Sweep(aPts, bPts []Vec2, va, vb Vec2) (*SweepResult, error) {
	a, err := NewPolygon(aPts)
	if err != nil {
		return nil, namedError("A", err.(*KernelError))
	}
	b, err := NewPolygon(bPts)
	if err != nil {
		return nil, namedError("B", err.(*KernelError))
	}
	if !isFinite(va.X) || !isFinite(va.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "velocity A: components must be finite numbers"}
	}
	if !isFinite(vb.X) || !isFinite(vb.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "velocity B: components must be finite numbers"}
	}
	// Positions are affine in t, so the coordinate extremes over the window
	// sit at t=0 (already validated) and t=1: reject motions that overflow
	// the finite coordinate range inside the window.
	for _, p := range aPts {
		if !isFinite(p.X+va.X) || !isFinite(p.Y+va.Y) {
			return nil, &KernelError{Code: ErrNonFiniteCoordinate,
				Message: "velocity A: moves polygon A to non-finite coordinates within the time window"}
		}
	}
	for _, p := range bPts {
		if !isFinite(p.X+vb.X) || !isFinite(p.Y+vb.Y) {
			return nil, &KernelError{Code: ErrNonFiniteCoordinate,
				Message: "velocity B: moves polygon B to non-finite coordinates within the time window"}
		}
	}

	contactTol := sweepContactRel * math.Max(1, math.Max(a.maxLen(), b.maxLen()))
	vrel := vb.Sub(va)
	// The relative velocity drives the advancement; it can overflow to
	// non-finite even when both velocities are finite individually.
	if !isFinite(vrel.X) || !isFinite(vrel.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "relative velocity (velocity B - velocity A) is not finite"}
	}

	// evalAt is the single-frame kernel evaluated on the poses at time t.
	evalAt := func(t float64) (*Result, error) {
		return evaluatePair(translatePolygon(a, va.Scale(t)), translatePolygon(b, vb.Scale(t)))
	}

	res0, err := evalAt(0)
	if err != nil {
		return nil, err
	}
	// Boundary case: already overlapping at t=0. The first-contact time is
	// 0 by definition and the contact information is the static penetration
	// verdict at the initial pose.
	if res0.Status == StatusPenetrated {
		return &SweepResult{
			Status:           StatusImpact,
			Time:             0,
			PenetrationDepth: res0.PenetrationDepth,
			Normal:           res0.Normal,
			PointA:           res0.PointA,
			PointB:           res0.PointB,
		}, nil
	}
	// Boundary case: no relative motion (common velocity or both at rest).
	// The configuration is frozen, so the sweep verdict over the whole
	// window is exactly the static verdict at the initial pose.
	if vrel.X == 0 && vrel.Y == 0 {
		return &SweepResult{
			Status:   StatusClear,
			Time:     0,
			Distance: res0.Distance,
			Normal:   res0.Normal,
			PointA:   res0.PointA,
			PointB:   res0.PointB,
		}, nil
	}

	t := 0.0
	res := res0
	for steps := 1; ; steps++ {
		if steps > sweepMaxSteps {
			return nil, &KernelError{Code: ErrSweepNoConvergence,
				Message: "sweep conservative advancement did not converge within the step budget"}
		}
		// Loop invariant: the parts are separated at time t and res is the
		// static frame at t.
		if res.Distance <= contactTol {
			return impactResult(t, res, steps), nil
		}
		c := -vrel.Dot(res.Normal)
		if c <= 0 {
			// The convex gap is non-decreasing from t onward, so no contact
			// can occur in [t,1]; the closest approach lies in [0,t].
			return clearResult(evalAt, vrel, steps)
		}
		dt := res.Distance / c
		if t+dt > 1 {
			// The tangent lower bound stays positive past the window end:
			// contact is only possible exactly at t=1.
			r1, err := evalAt(1)
			if err != nil {
				return nil, err
			}
			switch {
			case r1.Status == StatusPenetrated && r1.PenetrationDepth > contactTol:
				// Genuine contact strictly inside (t,1]. The conservative
				// bound makes this unreachable in exact arithmetic; bracket
				// it honestly instead of guessing.
				return bisectImpact(evalAt, t, 1, res, steps)
			case r1.Status == StatusPenetrated || r1.Distance <= contactTol:
				// Touching (within contact tolerance) exactly at the
				// window end.
				return impactResult(1, r1, steps), nil
			default:
				return clearResult(evalAt, vrel, steps)
			}
		}
		tNext := t + dt
		if tNext <= t {
			return nil, &KernelError{Code: ErrSweepNoConvergence,
				Message: "sweep advancement stalled: the time step underflows the representable grid"}
		}
		rn, err := evalAt(tNext)
		if err != nil {
			return nil, err
		}
		if rn.Status == StatusPenetrated {
			if rn.PenetrationDepth <= contactTol {
				// Landed on the boundary within contact tolerance.
				return impactResult(tNext, rn, steps), nil
			}
			// Overshot into genuine penetration (float rounding of the
			// step): bracket [t, tNext] and pin the boundary by bisection.
			return bisectImpact(evalAt, t, tNext, res, steps)
		}
		t, res = tNext, rn
	}
}

// impactResult builds the contact verdict at the given time from the static
// frame at that instant (separated-within-tolerance or boundary penetration).
func impactResult(time float64, frame *Result, steps int) *SweepResult {
	return &SweepResult{
		Status:     StatusImpact,
		Time:       time,
		Normal:     frame.Normal,
		PointA:     frame.PointA,
		PointB:     frame.PointB,
		Iterations: steps,
	}
}

// clearResult builds the no-contact verdict: the closest-approach search
// locates the minimum-gap instant and its frame supplies gap, normal and
// closest points.
func clearResult(evalAt func(float64) (*Result, error), vrel Vec2, steps int) (*SweepResult, error) {
	ct, frame, err := closestApproach(evalAt, vrel)
	if err != nil {
		return nil, err
	}
	return &SweepResult{
		Status:     StatusClear,
		Time:       ct,
		Distance:   frame.Distance,
		Normal:     frame.Normal,
		PointA:     frame.PointA,
		PointB:     frame.PointB,
		Iterations: steps,
	}, nil
}

// bisectImpact pins the first-contact time inside the bracket [lo, hi]
// (separated at lo, penetrated at hi) and reports the contact using the
// separated-side frame, whose gap normal is the contact normal. The bracket
// width after the fixed iteration count is far below any contact tolerance.
func bisectImpact(evalAt func(float64) (*Result, error), lo, hi float64, resLo *Result, steps int) (*SweepResult, error) {
	for i := 0; i < sweepBisectSteps; i++ {
		mid := lo + (hi-lo)/2
		rm, err := evalAt(mid)
		if err != nil {
			return nil, err
		}
		if rm.Status == StatusPenetrated {
			hi = mid
		} else {
			lo, resLo = mid, rm
		}
	}
	return impactResult(lo+(hi-lo)/2, resLo, steps), nil
}

// closestApproach finds the time of minimum gap within [0,1] for a motion
// provably free of contact. The gap is convex in t with subgradient
// vrel·n(t); the minimum sits at an endpoint when the subgradient sign does
// not straddle zero, otherwise a sign-change bisection pins it. Endpoint
// minima (receding motions -> t=0, still-closing motions -> t=1) come out
// exact, not approximate.
func closestApproach(evalAt func(float64) (*Result, error), vrel Vec2) (float64, *Result, error) {
	r0, err := evalAt(0)
	if err != nil {
		return 0, nil, err
	}
	if r0.Status != StatusSeparated {
		return 0, nil, errInconsistentSweep
	}
	if vrel.Dot(r0.Normal) >= 0 {
		return 0, r0, nil
	}
	r1, err := evalAt(1)
	if err != nil {
		return 0, nil, err
	}
	if r1.Status != StatusSeparated {
		return 0, nil, errInconsistentSweep
	}
	if vrel.Dot(r1.Normal) <= 0 {
		return 1, r1, nil
	}
	lo, hi := 0.0, 1.0
	for i := 0; i < sweepBisectSteps; i++ {
		mid := lo + (hi-lo)/2
		rm, err := evalAt(mid)
		if err != nil {
			return 0, nil, err
		}
		if rm.Status != StatusSeparated {
			return 0, nil, errInconsistentSweep
		}
		if vrel.Dot(rm.Normal) < 0 {
			lo = mid
		} else {
			hi = mid
		}
	}
	t := lo + (hi-lo)/2
	rt, err := evalAt(t)
	if err != nil {
		return 0, nil, err
	}
	return t, rt, nil
}

// errInconsistentSweep is the defensive verdict for a state the solver
// invariants prove unreachable (penetration where the motion was shown
// clear). It is surfaced as an honest error, never as a "safe" answer.
var errInconsistentSweep = &KernelError{Code: ErrSweepNoConvergence,
	Message: "sweep reached an inconsistent state: penetration detected where the motion was proven clear"}

// translatePolygon returns a copy of p rigidly shifted by d. Translation
// preserves convexity, orientation and vertex validity, so the copy needs
// no re-validation; only the tolerance scale is recomputed.
func translatePolygon(p *Polygon, d Vec2) *Polygon {
	vs := make([]Vec2, len(p.Vertices))
	scale := 1.0
	for i, v := range p.Vertices {
		vs[i] = v.Add(d)
		if m := math.Max(math.Abs(vs[i].X), math.Abs(vs[i].Y)); m > scale {
			scale = m
		}
	}
	return &Polygon{Vertices: vs, CCW: p.CCW, scale: scale}
}
