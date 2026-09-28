// Conservative-advancement swept query over the static single-frame kernel.
//
// Let f(t) be the signed gap between the parts at time t (positive when
// separated, negative when penetrating). f is the distance from the origin to
// a Minkowski difference translated at constant relative velocity, hence a
// convex function of t. At a separated instant the static kernel gives the
// gap g = f(t) and a unit gap normal n pointing from the closest point on A
// toward the closest point on B. Differentiating the closest-feature distance
// gives the right derivative
//
//	f'_+(t) = (vB - vA)·n = w·n.
//
// Convexity gives the supporting-line bound
//
//	f(τ) ≥ f(t) + w·n·(τ - t).
//
// When the parts approach (w·n < 0), advancing by
//
//	Δt = g / -(w·n)
//
// is provably safe: the supporting line stays non-negative on [t, t+Δt], so
// the parts cannot penetrate before t+Δt, which is a lower bound on the true
// time of impact. Repeating from the new witness feature converges to the
// real first zero of f — this is root finding driven by the kernel's gap and
// gap direction, never a sampling of the time window.
//
// When w·n ≥ 0 the convex gap can no longer decrease, so the motion is safe;
// the closest instant (a stationary point of f, where the approach rate
// crosses zero) is bracketed between successive advancement instants and
// refined by bisection on the sign of w·n.
package geometry

import "math"

// MaxSweepIterations bounds the number of conservative-advancement steps
// (single-frame evaluations are bounded separately inside the final
// bisection). Exhausting it is an error: no "safe" answer is fabricated when
// the root did not converge.
const MaxSweepIterations = 64

// sweepMaxSteps is the live bound, indirected so tests can force exhaustion.
var sweepMaxSteps = MaxSweepIterations

// sweepTimeTol is the absolute time tolerance to which an impact time or a
// closest-approach instant is refined. Bisection reaches it in well under 64
// halvings of a window of length 1 (2^-44 < 6e-14).
const sweepTimeTol = 1e-13

// bisectMaxSteps bounds either bisection refinement (root or minimizer).
const bisectMaxSteps = 64

type sweeper struct {
	a, b       *Polygon
	oa, ob     Vec2
	va, vb     Vec2
	rel        Vec2
	contactTol float64

	// frames counts distinct single-frame Evaluate calls.
	frames int
	// cache memoizes frames by the exact time argument.
	cache map[float64]*Result
}

func (s *sweeper) run() (*SweepResult, error) {
	s.rel = s.vb.Sub(s.va) // gap derivative uses B relative to A
	s.contactTol = sweepContactTol(s.poseScale())
	// A projection of the relative velocity on the gap normal this close to
	// zero counts as "not approaching"; it is dimensionless in speed.
	rateFlat := 1e-12 * math.Max(1.0, s.rel.Len())

	r0, err := s.frame(0)
	if err != nil {
		return nil, err
	}

	// Relative velocity identically zero: the relative geometry is frozen.
	// This is checked BEFORE the contact-tolerance shortcut so the swept
	// answer is exactly one static evaluation at the initial poses, whatever
	// the static kernel reports (separated gap or penetration).
	if s.rel == (Vec2{}) {
		if r0.Status == StatusPenetrated {
			return s.contactResult(0, r0), nil
		}
		return s.safeResult(0, r0), nil
	}

	// Boundary 1: already pressed together (penetrating, or separated by less
	// than the contact tolerance) at the start of the window.
	if s.isClosed(r0) {
		return s.contactResult(0, r0), nil
	}

	// Boundary 3: moving apart from the first instant. A convex gap whose
	// right derivative at 0 is non-negative can only grow; closest time is 0.
	if s.approachRate(r0) >= -rateFlat {
		return s.safeResult(0, r0), nil
	}

	t := 0.0
	r := r0
	// tPrev/rPrev is the preceding advancement iterate (known to still be
	// approaching); it brackets the closest instant once the gap turns around.
	tPrev, rPrev := 0.0, r0
	for step := 0; step < sweepMaxSteps; step++ {
		rate := s.approachRate(r) // < 0: approaching along the gap normal
		g := r.Distance
		if rate >= -rateFlat {
			// Gap stopped decreasing. The minimizer lies no earlier than the
			// previous iterate tPrev (where it was still closing) and no later
			// than t; refine that bracket.
			return s.closestBetween(tPrev, rPrev, t, r)
		}

		dt := g / -rate
		tNew := t + dt
		if !(dt > 0) || tNew <= t {
			return nil, &KernelError{Code: ErrSweepNoConvergence,
				Message: "swept conservative advancement stopped making progress before reaching the contact tolerance"}
		}

		if tNew >= 1-sweepTimeTol {
			// The safe-advance bound reaches (or passes) the window end:
			// inspect the exact end pose, never a time beyond [0,1].
			r1, err := s.frame(1)
			if err != nil {
				return nil, err
			}
			if r1.Status == StatusPenetrated {
				// Contact happens strictly before t = 1: refine the true
				// first root between the last known-separated iterate and the
				// penetrating end frame instead of reporting t = 1.
				return s.bisectContact(t, r, 1, r1)
			}
			if r1.Distance <= s.contactTol {
				// Boundary 2a: first touch at (numerically) t = 1.
				return s.contactResult(1, r1), nil
			}
			// Boundary 2b: still separated at the end. Either the gap is
			// still closing (closest at t = 1) or the closest instant sits
			// inside the window and gets bracketed/refined.
			if s.approachRate(r1) <= -rateFlat {
				return s.safeResult(1, r1), nil
			}
			return s.closestBetween(t, r, 1, r1)
		}

		rNew, err := s.frame(tNew)
		if err != nil {
			return nil, err
		}
		if s.isClosed(rNew) {
			// The true first zero is bracketed in [t, tNew]. Refine it.
			return s.bisectContact(t, r, tNew, rNew)
		}

		// Still separated at tNew: keep the approaching iterate as the lower
		// bracket and continue advancing from the new witness feature.
		tPrev, rPrev = t, r
		t, r = tNew, rNew
	}
	return nil, &KernelError{Code: ErrSweepNoConvergence,
		Message: "swept conservative advancement did not converge within the step budget"}
}

// frame evaluates the single-frame kernel at time t, translating each part to
// its world pose. Results are memoized by t. This is the ONLY geometric
// primitive the swept layer uses.
func (s *sweeper) frame(t float64) (*Result, error) {
	if r, ok := s.cache[t]; ok {
		return r, nil
	}
	da := s.oa.Add(s.va.Scale(t))
	db := s.ob.Add(s.vb.Scale(t))
	aPts := make([]Vec2, len(s.a.Vertices))
	bPts := make([]Vec2, len(s.b.Vertices))
	for i, v := range s.a.Vertices {
		aPts[i] = v.Add(da)
	}
	for i, v := range s.b.Vertices {
		bPts[i] = v.Add(db)
	}
	r, err := Evaluate(aPts, bPts)
	if err != nil {
		return nil, err
	}
	s.frames++
	s.cache[t] = r
	return r, nil
}

// isClosed reports whether a frame is already within the contact tolerance
// (penetrating, or separated by a sub-tolerance residual gap).
func (s *sweeper) isClosed(r *Result) bool {
	return r.Status == StatusPenetrated || r.Distance <= s.contactTol
}

// approachRate returns f'_+(t) = (vB-vA)·n at a separated frame; negative
// means the gap closes along the current gap normal.
func (s *sweeper) approachRate(r *Result) float64 { return s.rel.Dot(r.Normal) }

// bisectContact refines the first zero of the gap inside [lo, hi], where lo
// is a known-separated instant and hi a known-closed instant, using plain
// sign bisection on the single-frame kernel. It converges to the true time of
// impact regardless of the conservative-advancement step size:
//
//   - a midpoint reported as penetrated (genuine sign change of the gap) keeps
//     shrinking the bracket down to sweepTimeTol, so the reported time is
//     bounded by bracket width rather than by the gap tolerance divided by
//     closing speed;
//   - a midpoint merely separated by a sub-tolerance residual is accepted as
//     the numerical contact face (this only happens within kernel tolerance
//     of the root).
func (s *sweeper) bisectContact(lo float64, rLo *Result, hi float64, rHi *Result) (*SweepResult, error) {
	for i := 0; i < bisectMaxSteps; i++ {
		if hi-lo <= sweepTimeTol {
			break
		}
		mid := 0.5 * (lo + hi)
		rm, err := s.frame(mid)
		if err != nil {
			return nil, err
		}
		switch {
		case rm.Status == StatusPenetrated:
			hi, rHi = mid, rm
		case rm.Distance <= s.contactTol:
			return s.contactResult(mid, rm), nil
		default:
			lo, rLo = mid, rm
		}
	}
	// Report the geometry at the bracket midpoint (re-evaluated), so the
	// witnesses correspond to the reported time rather than to an arbitrary
	// early penetrating bracket end.
	t := 0.5 * (lo + hi)
	rt, err := s.frame(t)
	if err != nil {
		return nil, err
	}
	return s.contactResult(t, rt), nil
}

// closestBetween refines the minimizer of the gap over a bracket on which the
// approach rate crosses from negative (rLo, gap decreasing) to non-negative
// (rHi, gap non-decreasing), by bisecting on the sign of w·n.
func (s *sweeper) closestBetween(lo float64, rLo *Result, hi float64, rHi *Result) (*SweepResult, error) {
	if s.approachRate(rLo) >= 0 {
		return s.safeResult(lo, rLo), nil
	}
	if s.approachRate(rHi) < 0 {
		return s.safeResult(hi, rHi), nil
	}
	for i := 0; i < bisectMaxSteps; i++ {
		if hi-lo <= sweepTimeTol {
			break
		}
		mid := 0.5 * (lo + hi)
		rm, err := s.frame(mid)
		if err != nil {
			return nil, err
		}
		if s.approachRate(rm) < 0 {
			lo, rLo = mid, rm
		} else {
			hi, rHi = mid, rm
		}
	}
	t := 0.5 * (lo + hi)
	rt, err := s.frame(t)
	if err != nil {
		return nil, err
	}
	return s.safeResult(t, rt), nil
}

func (s *sweeper) contactResult(t float64, r *Result) *SweepResult {
	res := &SweepResult{
		Status:           StatusContact,
		Time:             clampUnit(t),
		PenetrationDepth: r.PenetrationDepth,
		Normal:           r.Normal,
		PointA:           r.PointA,
		PointB:           r.PointB,
		Iterations:       s.frames,
	}
	return res
}

func (s *sweeper) safeResult(t float64, r *Result) *SweepResult {
	return &SweepResult{
		Status:     StatusSafe,
		Time:       clampUnit(t),
		Distance:   r.Distance,
		Normal:     r.Normal,
		PointA:     r.PointA,
		PointB:     r.PointB,
		Iterations: s.frames,
	}
}

// poseScale bounds the coordinate magnitude reached by either part over the
// window, used to scale the contact tolerance.
func (s *sweeper) poseScale() float64 {
	m := 0.0
	for _, t := range []float64{0, 1} {
		for _, v := range s.a.Vertices {
			p := v.Add(s.oa.Add(s.va.Scale(t)))
			m = math.Max(m, math.Max(math.Abs(p.X), math.Abs(p.Y)))
		}
		for _, v := range s.b.Vertices {
			p := v.Add(s.ob.Add(s.vb.Scale(t)))
			m = math.Max(m, math.Max(math.Abs(p.X), math.Abs(p.Y)))
		}
	}
	return m
}
