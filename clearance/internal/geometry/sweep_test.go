package geometry

import (
	"math"
	"math/rand"
	"testing"
)

// Numerical acceptance criteria for swept queries, stated explicitly:
const (
	// analyticTOITol: first-contact time must match the closed-form root.
	// 1e-9 is roughly the single-frame kernel tolerance (eps*scale divided by
	// closing speed) at unit coordinate scale — orders of magnitude tighter
	// than any time-window sampling could achieve.
	analyticTOITol = 1e-9
	// toiTightTol: end-window contact time ("exactly t=1") tolerance.
	toiTightTol = 1e-9
	// sweepVecTol: agreement of normals / points with analytic witnesses.
	sweepVecTol = 1e-9
	// safeTimeTol: closest-time reporting for safe motions.
	safeTimeTol = 1e-10
)

// motion builds a Motion whose polygon already carries its initial world pose
// (zero offset), with the given constant velocity.
func motion(pts []Vec2, v Vec2) Motion {
	return Motion{Vertices: pts, Velocity: v}
}

// TestSweep_HeadOn_AnalyticTOI: axis-aligned squares, A fixed, B approaching
// head-on. Gap g0 = 0.5, closing speed = 2 -> analytic TOI = 0.25, contact
// normal = +X, contact on the facing edges.
func TestSweep_HeadOn_AnalyticTOI(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.5, 0, 2.5, 1) // 0.5 gap to A's right edge x=1
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -2}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if !approxEq(res.Time, 0.25, analyticTOITol) {
		t.Fatalf("TOI = %.15g, want 0.25", res.Time)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, sweepVecTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if !approxEq(res.Normal.Len(), 1, 1e-12) {
		t.Fatalf("normal not unit: %v", res.Normal)
	}
	// At TOI the facing edges coincide at x = 1; witnesses realize zero gap.
	if !approxEq(res.PointA.X, 1, 1e-7) || !approxEq(res.PointB.X, 1, 1e-7) {
		t.Fatalf("contact points off facing edges: A=%v B=%v", res.PointA, res.PointB)
	}
	if gap := res.PointA.Sub(res.PointB).Len(); gap > 1e-7 {
		t.Fatalf("witness gap at TOI = %.3e, want ~0", gap)
	}
	if !approxEq(res.PointA.Y, res.PointB.Y, 1e-7) {
		t.Fatalf("witness points not aligned: %v vs %v", res.PointA, res.PointB)
	}
	if res.PenetrationDepth > 1e-7 {
		t.Fatalf("first-contact depth = %.3e, want ~0", res.PenetrationDepth)
	}
	if res.Time < 0 || res.Time > 1 {
		t.Fatalf("TOI outside [0,1]: %v", res.Time)
	}
}

// TestSweep_HeadOn_Table covers several analytically solvable approaches on
// both axes, including both parts moving at once and an oblique triangle hit.
func TestSweep_HeadOn_Table(t *testing.T) {
	cases := []struct {
		name    string
		a, b    []Vec2
		va, vb  Vec2
		wantTOI float64
		wantN   Vec2
	}{
		{
			name:    "y axis, both close at speed 1 each, gap 1",
			a:       rect(0, 0, 1, 1),
			b:       rect(0, 2, 1, 3),
			va:      Vec2{Y: 1},
			vb:      Vec2{Y: -1},
			wantTOI: 0.5,
			wantN:   Vec2{0, 1},
		},
		{
			name:    "x axis, A chases B, gap 1, relative speed 1, touch at t=1",
			a:       rect(0, 0, 1, 1),
			b:       rect(2, 0, 3, 1),
			va:      Vec2{X: 4},
			vb:      Vec2{X: 3},
			wantTOI: 1.0,
			wantN:   Vec2{1, 0},
		},
		{
			// Two right triangles face along parallel hypotenuses. A's
			// hypotenuse is x+y=1 with outward normal n=(1,1)/sqrt2; B (same
			// shape) is translated so its lower-left vertex sits at
			// (0.6,0.8) = 0.05*(6,8) = (sqrt2+0.2)*n*..., concretely the
			// initial closest pair is B's vertex (0.6,0.8) to A's hypotenuse
			// with gap g0 = (0.6+0.8-1)/sqrt2 = 0.4/sqrt2 and normal n.
			// Relative motion vB=(-3,-4) is along n (n·(-3,-4) = -7/sqrt2),
			// so f(t) is linear in that feature: TOI = 0.4/7 = 2/35.
			name:    "oblique triangle approach, vertex to hypotenuse",
			a:       tri(Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}),
			b:       tri(Vec2{0.6, 0.8}, Vec2{1.6, 0.8}, Vec2{0.6, 1.8}),
			va:      Vec2{},
			vb:      Vec2{X: -3, Y: -4},
			wantTOI: 2.0 / 35.0,
			wantN:   Vec2{1.0 / math.Sqrt2, 1.0 / math.Sqrt2},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := Sweep(motion(tc.a, tc.va), motion(tc.b, tc.vb))
			if err != nil {
				t.Fatalf("sweep: %v", err)
			}
			if res.Status != StatusContact {
				t.Fatalf("status = %s, want contact", res.Status)
			}
			if !approxEq(res.Time, tc.wantTOI, analyticTOITol) {
				t.Fatalf("TOI = %.15g, want %.15g", res.Time, tc.wantTOI)
			}
			if !vecEq(res.Normal, tc.wantN, sweepVecTol) {
				t.Fatalf("normal = %v, want %v", res.Normal, tc.wantN)
			}
			if res.PointA.Sub(res.PointB).Len() > 1e-7 {
				t.Fatalf("witnesses not touching at TOI: %v vs %v", res.PointA, res.PointB)
			}
		})
	}
}

// TestSweep_Separating_IsSafe_ClosestAtZero is acceptance case 2: reversing
// the head-on velocity so the parts move apart must report a safe window with
// the closest instant at t = 0.
func TestSweep_Separating_IsSafe_ClosestAtZero(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.5, 0, 2.5, 1)
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: 2}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusSafe {
		t.Fatalf("status = %s, want safe", res.Status)
	}
	if !approxEq(res.Time, 0, safeTimeTol) {
		t.Fatalf("closest time = %.15g, want 0", res.Time)
	}
	if !approxEq(res.Distance, 0.5, analyticTOITol) {
		t.Fatalf("min distance = %.15g, want 0.5", res.Distance)
	}
	if !vecEq(res.Normal, Vec2{1, 0}, sweepVecTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if d := res.PointA.Sub(res.PointB).Len(); !approxEq(d, res.Distance, 1e-9) {
		t.Fatalf("witness distance %.12g != reported %.12g", d, res.Distance)
	}
}

// TestSweep_CornerMiss_InteriorClosest: the parts initially close (rate < 0)
// but the gap bottoms out at an interior instant and they miss. B = unit
// square starting 1.5 to the right, relative velocity (1,-3) in CSO terms
// (vB = (-1,3)): it rises over A's top-right corner. The closest feature is
// the corner pair A(1,1) / B(1.5-t,3t) with
//
//	d(t) = (0.5-t, -1+3t),  d·d' = -3.5 + 10t = 0  ->  t* = 0.35,
//	d(0.35) = (0.15,0.05),   gap = sqrt(0.025) > 0,
//	n = (0.15,0.05)/sqrt(.025) = (3,1)/sqrt(10).
func TestSweep_CornerMiss_InteriorClosest(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.5, 0, 2.5, 1)
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -1, Y: 3}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusSafe {
		t.Fatalf("status = %s, want safe (miss)", res.Status)
	}
	if !approxEq(res.Time, 0.35, 1e-8) {
		t.Fatalf("closest time = %.12g, want 0.35", res.Time)
	}
	wantGap := math.Sqrt(0.025)
	if !approxEqRel(res.Distance, wantGap, 1e-9, 1e-11) {
		t.Fatalf("min gap = %.12g, want %.12g", res.Distance, wantGap)
	}
	sqrt10 := math.Sqrt(10)
	wantN := Vec2{X: 3 / sqrt10, Y: 1 / sqrt10}
	if !vecEq(res.Normal, wantN, 1e-7) {
		t.Fatalf("normal = %v, want %v", res.Normal, wantN)
	}
	// At the minimizer the approach rate (vB-vA)·n must be numerically zero.
	if rn := (Vec2{X: -1, Y: 3}).Dot(res.Normal); math.Abs(rn) > 1e-6 {
		t.Fatalf("(vB-vA)·n = %.3e not ~0 at reported closest instant", rn)
	}
	if d := res.PointA.Sub(res.PointB).Len(); !approxEqRel(d, wantGap, 1e-7, 1e-9) {
		t.Fatalf("witness distance %.10g != reported %.10g", d, res.Distance)
	}
	// Independently scan poses densely: none may penetrate, and the reported
	// gap must not be beaten by more than numerical noise.
	for k := 0; k <= 2000; k++ {
		tk := float64(k) / 2000
		r, err := Evaluate(a, translate(b, Vec2{X: -1, Y: 3}.Scale(tk)))
		if err != nil {
			t.Fatalf("scan t=%v: %v", tk, err)
		}
		if r.Status != StatusSeparated {
			t.Fatalf("scan found %s at t=%v, fixture is supposed to miss", r.Status, tk)
		}
		if r.Distance < res.Distance-1e-9 {
			t.Fatalf("scan t=%v gap %.12g beats reported minimum %.12g", tk, r.Distance, res.Distance)
		}
	}
}

// TestSweep_CommonTranslationAndVelocityInvariant is acceptance case 3:
// adding the same rigid offset AND the same common velocity to both parts
// leaves TOI, normal and relative contact geometry unchanged.
func TestSweep_CommonTranslationAndVelocityInvariant(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.5, 0, 2.5, 1)
	base, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -2}))
	if err != nil {
		t.Fatalf("base: %v", err)
	}

	shifts := []Vec2{{X: 10, Y: -7}, {X: -100, Y: 250.5}, {X: 0.333, Y: 1.25}}
	commonVels := []Vec2{{X: 3, Y: 4}, {X: -50, Y: 12.5}, {X: 1000, Y: -2000}}
	for i := range shifts {
		sh, w := shifts[i], commonVels[i]
		ma := Motion{Vertices: a, Offset: sh, Velocity: w}
		mb := Motion{Vertices: b, Offset: sh, Velocity: Vec2{X: -2}.Add(w)}

		// Numerical floor for this variant: the single-frame kernel's gap
		// resolution is eps*S where S is the largest world-coordinate
		// magnitude reached. The TOI face therefore carries a time floor of
		// eps*S / |relative closing speed| (here |vrel| = 2).
		S := motionWorldScale(ma, mb)
		toiTol := math.Max(analyticTOITol, 1e-10*S/2.0)

		res, err := Sweep(ma, mb)
		if err != nil {
			t.Fatalf("shift %v common velocity %v: %v", sh, w, err)
		}
		if res.Status != base.Status {
			t.Fatalf("status changed: %s vs %s", res.Status, base.Status)
		}
		if !approxEq(res.Time, base.Time, toiTol) {
			t.Fatalf("TOI changed under common motion: %.15g vs %.15g (tol %.3g)",
				res.Time, base.Time, toiTol)
		}
		if !vecEq(res.Normal, base.Normal, math.Max(sweepVecTol, 1e-10*S)) {
			t.Fatalf("normal changed under common motion: %v vs %v", res.Normal, base.Normal)
		}
		// Contact points translate by exactly the common world displacement.
		// The unavoidable mismatch is the gap-resolution floor plus the
		// common speed times the (measured) time discrepancy between the two
		// independently converged impact times.
		disp := sh.Add(w.Scale(base.Time))
		pointTol := 1e-10*S + w.Len()*math.Abs(res.Time-base.Time) + 1e-9
		if !vecEq(res.PointA, base.PointA.Add(disp), pointTol) {
			t.Fatalf("pointA = %v, want %v (tol %.3g)", res.PointA, base.PointA.Add(disp), pointTol)
		}
		if !vecEq(res.PointB, base.PointB.Add(disp), pointTol) {
			t.Fatalf("pointB = %v, want %v (tol %.3g)", res.PointB, base.PointB.Add(disp), pointTol)
		}
	}
}

// motionWorldScale mirrors the kernel's coordinate-magnitude scale used for
// tolerance scaling, over both parts at both window ends.
func motionWorldScale(ms ...Motion) float64 {
	S := 1.0
	consider := func(p Vec2) {
		S = math.Max(S, math.Max(math.Abs(p.X), math.Abs(p.Y)))
	}
	for _, m := range ms {
		for _, tk := range []float64{0, 1} {
			d := m.Offset.Add(m.Velocity.Scale(tk))
			for _, v := range m.Vertices {
				consider(v.Add(d))
			}
		}
	}
	return S
}

// TestSweep_ZeroRelativeVelocity_MatchesStatic is acceptance case 4.
func TestSweep_ZeroRelativeVelocity_MatchesStatic(t *testing.T) {
	cases := []struct {
		name   string
		a, b   []Vec2
		common Vec2
	}{
		{"both stationary, separated", rect(0, 0, 2, 1), rect(2.75, -0.5, 4.25, 1.5), Vec2{}},
		{"both translate together, separated", rect(0, 0, 2, 1), rect(2.75, -0.5, 4.25, 1.5), Vec2{3.5, -1.25}},
		{"both stationary, edge touching", rect(0, 0, 1, 1), rect(1, 0, 2, 1), Vec2{}},
		{"both translate together, penetrated", rect(0, 0, 2, 1), rect(1.7, 0.1, 3.7, 1.9), Vec2{2, -3}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			static, err := Evaluate(tc.a, tc.b)
			if err != nil {
				t.Fatalf("static: %v", err)
			}
			res, err := Sweep(motion(tc.a, tc.common), motion(tc.b, tc.common))
			if err != nil {
				t.Fatalf("sweep: %v", err)
			}
			if res.Time != 0 {
				t.Fatalf("time = %v, want 0", res.Time)
			}
			if static.Status == StatusSeparated {
				if res.Status != StatusSafe {
					t.Fatalf("status = %s, want safe matching static separated", res.Status)
				}
				if !approxEqRel(res.Distance, static.Distance, 1e-12, 1e-12) {
					t.Fatalf("distance %.15g != static %.15g", res.Distance, static.Distance)
				}
			} else {
				if res.Status != StatusContact {
					t.Fatalf("status = %s, want contact matching static penetrated", res.Status)
				}
				if !approxEqRel(res.PenetrationDepth, static.PenetrationDepth, 1e-12, 1e-12) {
					t.Fatalf("depth %.15g != static %.15g", res.PenetrationDepth, static.PenetrationDepth)
				}
			}
			if !vecEq(res.Normal, static.Normal, 1e-12) {
				t.Fatalf("normal %v != static %v", res.Normal, static.Normal)
			}
			if !vecEq(res.PointA, static.PointA, 1e-12) || !vecEq(res.PointB, static.PointB, 1e-12) {
				t.Fatalf("points %v/%v != static %v/%v", res.PointA, res.PointB, static.PointA, static.PointB)
			}
		})
	}
}

// TestSweep_InitialPenetration is acceptance case 1: overlapping at t=0 gives
// TOI 0 with the static penetration geometry, even though the applied
// velocity would separate the parts later in the window.
func TestSweep_InitialPenetration(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(1.7, 0.1, 3.7, 1.9) // x overlap 0.3, static depth 0.3
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("static: %v", err)
	}
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: 5, Y: 5}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if res.Time != 0 {
		t.Fatalf("TOI = %v, want 0", res.Time)
	}
	if !approxEq(res.PenetrationDepth, static.PenetrationDepth, 1e-12) {
		t.Fatalf("depth %.15g != static %.15g", res.PenetrationDepth, static.PenetrationDepth)
	}
	if !vecEq(res.Normal, static.Normal, 1e-12) {
		t.Fatalf("normal %v != static %v", res.Normal, static.Normal)
	}
	if !vecEq(res.PointA, static.PointA, 1e-12) {
		t.Fatalf("pointA %v != static %v", res.PointA, static.PointA)
	}
	if !vecEq(res.PointB, static.PointB, 1e-12) {
		t.Fatalf("pointB %v != static %v", res.PointB, static.PointB)
	}
}

// TestSweep_ContactExactlyAtWindowEnd is boundary 2a: parts close the whole
// window and touch exactly at t = 1.
func TestSweep_ContactExactlyAtWindowEnd(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1) // gap 1, closing speed 1 -> TOI exactly 1
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -1}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if !approxEq(res.Time, 1, toiTightTol) {
		t.Fatalf("TOI = %.15g, want 1", res.Time)
	}
	if !vecEq(res.Normal, Vec2{1, 0}, sweepVecTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweep_ContactJustBeforeWindowEnd: the conservative bound overshoots the
// window end and the end frame already overlaps. The reported TOI must be the
// true first root (10/11), NOT a sloppy t = 1.
func TestSweep_ContactJustBeforeWindowEnd(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1) // gap 1
	// Relative closing speed 1.1 -> TOI = 1/1.1 = 10/11 < 1, end frame deep
	// in penetration by 0.1.
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -1.1}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	wantTOI := 10.0 / 11.0
	if !approxEq(res.Time, wantTOI, analyticTOITol) {
		t.Fatalf("TOI = %.15g, want %.15g (must not be reported as 1)", res.Time, wantTOI)
	}
	if res.Time > 1-1e-6 {
		t.Fatalf("TOI %.15g suspiciously pinned to window end", res.Time)
	}
	if !vecEq(res.Normal, Vec2{1, 0}, sweepVecTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweep_NearMissAtWindowEnd is boundary 2b: still a hair short of contact
// at t = 1 and still closing -> safe, closest time exactly 1, residual gap
// equal to the analytic shortfall. Pinned with an explicit tolerance because
// this is the boundary advancement/convergence criteria get wrong.
func TestSweep_NearMissAtWindowEnd(t *testing.T) {
	const shortfall = 1e-6
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1)
	res, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -(1 - shortfall)}))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusSafe {
		t.Fatalf("status = %s, want safe", res.Status)
	}
	if !approxEq(res.Time, 1, safeTimeTol) {
		t.Fatalf("closest time = %.15g, want 1", res.Time)
	}
	if !approxEqRel(res.Distance, shortfall, 1e-6, 1e-11) {
		t.Fatalf("residual gap = %.12g, want %.3g", res.Distance, shortfall)
	}
}

// TestSweep_ContinuousCatchesWhatSamplingMisses is acceptance case 6, the
// anti-discretization proof. B crosses A inside a narrow contact interval
// that contains no grid point of a uniform 8-interval sampling:
//
//	A = [0,1]² fixed; B = [14,15]×[0,1] moving left at speed 50.
//	First contact at t = 13/50 = 0.26; B has passed completely through at
//	t = 15/50 = 0.30, so penetration holds only on (0.26,0.30). None of the
//	grid points k/8 (0, .125, .25, .375, .5, …) lies in that interval: a
//	sample-based checker reports "safe everywhere". The continuous solver
//	must report contact at the analytic TOI 0.26.
func TestSweep_ContinuousCatchesWhatSamplingMisses(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(14, 0, 15, 1) // initial gap 13
	const speed = 50.0
	va, vb := Vec2{}, Vec2{X: -speed}

	// Independent, deliberately crude reference: uniform static sampling.
	const segments = 8
	for k := 0; k <= segments; k++ {
		tk := float64(k) / float64(segments)
		r, err := Evaluate(a, translate(b, vb.Scale(tk)))
		if err != nil {
			t.Fatalf("static at t=%v: %v", tk, err)
		}
		if r.Status != StatusSeparated {
			t.Fatalf("fixture broken: grid point t=%v reports %s", tk, r.Status)
		}
	}

	res, err := Sweep(motion(a, va), motion(b, vb))
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusContact {
		t.Fatalf("continuous solver missed the narrow contact: status = %s", res.Status)
	}
	wantTOI := 13.0 / speed
	if !approxEq(res.Time, wantTOI, analyticTOITol) {
		t.Fatalf("TOI = %.15g, want %.15g", res.Time, wantTOI)
	}
	if !vecEq(res.Normal, Vec2{1, 0}, sweepVecTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if res.PointA.Sub(res.PointB).Len() > 1e-7 {
		t.Fatalf("contact points not coincident at TOI: %v vs %v", res.PointA, res.PointB)
	}
}

// TestSweep_IterationsBounded ensures the sweep terminates well within its
// budget over many relative speeds and reports the frames actually consumed.
func TestSweep_IterationsBounded(t *testing.T) {
	a := rect(0, 0, 1, 1)
	for i := 0; i < 200; i++ {
		b := rect(2.5, 0, 3.5, 1)
		res, err := Sweep(
			motion(a, Vec2{X: float64(i) * 0.01}),
			motion(b, Vec2{X: -float64(i)*0.03 - 0.5, Y: float64(i%7) * 0.02}),
		)
		if err != nil {
			t.Fatalf("i=%d: %v", i, err)
		}
		if res.Iterations <= 0 {
			t.Fatalf("i=%d: frame count not reported", i)
		}
		if res.Iterations > MaxSweepIterations+2*bisectMaxSteps+16 {
			t.Fatalf("i=%d consumed %d frames, exceeds budget", i, res.Iterations)
		}
		if res.Status == StatusContact && (res.Time < -1e-12 || res.Time > 1+1e-12) {
			t.Fatalf("i=%d TOI out of range: %v", i, res.Time)
		}
	}
}

// TestSweep_BudgetExhaustedReportsError: forcing the step budget to zero must
// surface SWEEP_NO_CONVERGENCE for a genuinely approaching pair; it must not
// be fabricated into a "safe" answer.
func TestSweep_BudgetExhaustedReportsError(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.5, 0, 2.5, 1)
	orig := sweepMaxSteps
	sweepMaxSteps = 0
	_, err := Sweep(motion(a, Vec2{}), motion(b, Vec2{X: -2}))
	sweepMaxSteps = orig
	assertCode(t, err, ErrSweepNoConvergence)
}

// TestSweep_InvalidInputs mirrors the static validation rules and adds the
// velocity-specific rejection.
func TestSweep_InvalidInputs(t *testing.T) {
	good := rect(0, 0, 1, 1)
	fin := Vec2{}

	cases := []struct {
		name string
		a, b Motion
		code string
	}{
		{"too few vertices A", Motion{Vertices: []Vec2{{0, 0}, {1, 1}}, Velocity: fin}, Motion{Vertices: good, Velocity: fin}, ErrTooFewVertices},
		{"degenerate B", Motion{Vertices: good, Velocity: fin}, Motion{Vertices: []Vec2{{0, 0}, {1, 1}, {2, 2}}, Velocity: fin}, ErrDegeneratePolygon},
		{"concave A", Motion{Vertices: []Vec2{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}, Velocity: fin}, Motion{Vertices: good, Velocity: fin}, ErrNonConvexPolygon},
		{"NaN vertex coordinate", Motion{Vertices: []Vec2{{0, 0}, {1, 0}, {1, math.NaN()}}, Velocity: fin}, Motion{Vertices: good, Velocity: fin}, ErrNonFiniteCoordinate},
		{"NaN velocity A", Motion{Vertices: good, Velocity: Vec2{X: math.NaN()}}, Motion{Vertices: good, Velocity: fin}, ErrNonFiniteVelocity},
		{"+Inf velocity B", Motion{Vertices: good, Velocity: fin}, Motion{Vertices: good, Velocity: Vec2{Y: math.Inf(1)}}, ErrNonFiniteVelocity},
		{"-Inf velocity B", Motion{Vertices: good, Velocity: fin}, Motion{Vertices: good, Velocity: Vec2{X: math.Inf(-1)}}, ErrNonFiniteVelocity},
		{"NaN offset A", Motion{Vertices: good, Offset: Vec2{X: math.NaN()}, Velocity: fin}, Motion{Vertices: good, Velocity: fin}, ErrNonFiniteCoordinate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := Sweep(tc.a, tc.b)
			assertCode(t, err, tc.code)
		})
	}
}

// TestSweep_RandomMatchesDenseOracle fuzzes the solver over 200 random
// convex pairs and relative motions, comparing the contact/SAFE verdict,
// impact time and minimum gap against an independent dense static scan (a
// test oracle deliberately separate from the product solver).
func TestSweep_RandomMatchesDenseOracle(t *testing.T) {
	rng := rand.New(rand.NewSource(20260928))
	for iter := 0; iter < 200; iter++ {
		a := randomConvex(rng, 3+rng.Intn(4), 1.5)
		b0 := randomConvex(rng, 3+rng.Intn(4), 1.0)
		// Start B at a moderate distance in a random direction.
		theta := rng.Float64() * 2 * math.Pi
		d := 2.0 + rng.Float64()*4.0
		b := translate(b0, Vec2{X: d * math.Cos(theta), Y: d * math.Sin(theta)})
		// Relative velocity: magnitude up to ~1.4d so contact may or may not
		// occur within the window; arbitrary direction (grazing included).
		speed := d * (0.2 + rng.Float64()*1.2)
		phi := theta + (rng.Float64()-0.5)*math.Pi
		va := Vec2{X: rng.NormFloat64(), Y: rng.NormFloat64()}
		vb := va.Add(Vec2{X: speed * math.Cos(phi), Y: speed * math.Sin(phi)})

		res, err := Sweep(motion(a, va), motion(b, vb))
		if err != nil {
			t.Fatalf("iter %d: sweep: %v", iter, err)
		}

		// Independent oracle: fine grid of static evaluations.
		const N = 4001
		oracleMinGap := math.Inf(1)
		oracleMinT := 0.0
		firstPen := -1.0
		for k := 0; k <= N; k++ {
			tk := float64(k) / float64(N)
			aw := translate(a, va.Scale(tk))
			bw := translate(b, vb.Scale(tk))
			r, err := Evaluate(aw, bw)
			if err != nil {
				t.Fatalf("iter %d oracle t=%v: %v", iter, tk, err)
			}
			if r.Status == StatusPenetrated {
				firstPen = tk
				break
			}
			if r.Distance < oracleMinGap {
				oracleMinGap = r.Distance
				oracleMinT = tk
			}
		}

		switch {
		case firstPen >= 0:
			if res.Status != StatusContact {
				t.Fatalf("iter %d: oracle penetrates by t=%.5f, sweep says safe", iter, firstPen)
			}
			// Grid first-penetration time brackets the true TOI: it is in
			// [firstPen-1/N, firstPen] (the grid cell width is 1/N = 2.5e-4).
			cell := 1.0 / float64(N)
			if res.Time < firstPen-cell-1e-9 || res.Time > firstPen+cell+1e-7 {
				t.Fatalf("iter %d: TOI %.9f outside oracle bracket [%.5f,%.5f]",
					iter, res.Time, firstPen-cell, firstPen)
			}
			if math.Abs(res.Normal.Len()-1) > 1e-9 {
				t.Fatalf("iter %d: contact normal not unit: %v", iter, res.Normal)
			}
		default:
			if res.Status != StatusSafe {
				t.Fatalf("iter %d: oracle never penetrates but sweep reports contact at %.6f",
					iter, res.Time)
			}
			if !approxEqRel(res.Distance, oracleMinGap, 1e-6, 1e-8) {
				t.Fatalf("iter %d: min gap %.9g != oracle %.9g", iter, res.Distance, oracleMinGap)
			}
			if math.Abs(res.Time-oracleMinT) > 5.0/float64(N)+1e-7 {
				t.Fatalf("iter %d: closest time %.6f disagrees with oracle %.6f",
					iter, res.Time, oracleMinT)
			}
		}
	}
}

// TestSweep_OffsetIsHonored verifies the initial-offset field places the
// local contour and reproduces the direct-vertices form exactly.
func TestSweep_OffsetIsHonored(t *testing.T) {
	local := rect(0, 0, 1, 1)
	direct, err := Sweep(motion(local, Vec2{}), motion(rect(1.5, 0, 2.5, 1), Vec2{X: -2}))
	if err != nil {
		t.Fatalf("direct: %v", err)
	}
	viaOffset, err := Sweep(
		Motion{Vertices: local, Velocity: Vec2{}},
		Motion{Vertices: local, Offset: Vec2{X: 1.5}, Velocity: Vec2{X: -2}},
	)
	if err != nil {
		t.Fatalf("offset: %v", err)
	}
	if !approxEq(viaOffset.Time, direct.Time, 1e-12) {
		t.Fatalf("offset TOI %.15g != direct %.15g", viaOffset.Time, direct.Time)
	}
	if !vecEq(viaOffset.Normal, direct.Normal, 1e-12) {
		t.Fatalf("offset normal %v != direct %v", viaOffset.Normal, direct.Normal)
	}
}
