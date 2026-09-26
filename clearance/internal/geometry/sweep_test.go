package geometry

import (
	"math"
	"math/rand"
	"strings"
	"testing"
)

// Sweep tolerances, stated explicitly up front (as everywhere in this
// package): toiTol bounds impact/closest-approach times; normals and gaps
// reuse the package-wide exactGapTol (1e-12, analytic cases) and
// absTol/relTol (1e-9).
const toiTol = 1e-9

// TestSweep_HeadOnClosedForm is acceptance case 1: two axis-aligned squares,
// one stationary, the other translating straight at it. The impact time has
// the closed form gap/closing-speed and must match to 1e-12; the contact
// normal is the motion axis.
func TestSweep_HeadOnClosedForm(t *testing.T) {
	a := rect(0, 0, 1, 1)
	cases := []struct {
		name       string
		b          []Vec2
		velA, velB Vec2
		wantTOI    float64
		wantNormal Vec2
		wantTouchX float64 // x of the touching faces at the impact time
	}{
		{"gap 2 speed 4", rect(3, 0, 4, 1), Vec2{}, Vec2{X: -4}, 0.5, Vec2{1, 0}, 1},
		{"gap 0.75 speed 1.5", rect(1.75, 0, 2.75, 1), Vec2{}, Vec2{X: -1.5}, 0.5, Vec2{1, 0}, 1},
		{"gap 3 speed 4", rect(4, 0, 5, 1), Vec2{}, Vec2{X: -4}, 0.75, Vec2{1, 0}, 1},
		{"both move, relative speed 4", rect(3, 0, 4, 1), Vec2{X: 1}, Vec2{X: -3}, 0.5, Vec2{1, 0}, 1.5},
		{"approach from the left", rect(-2, 0, -1, 1), Vec2{}, Vec2{X: 2}, 0.5, Vec2{-1, 0}, 0},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := Sweep(a, tc.b, tc.velA, tc.velB)
			if err != nil {
				t.Fatalf("sweep: %v", err)
			}
			if res.Verdict != VerdictContact {
				t.Fatalf("verdict = %s, want contact", res.Verdict)
			}
			if !approxEq(res.TimeOfImpact, tc.wantTOI, exactGapTol) {
				t.Fatalf("toi = %.15g, want %.15g (gap/closing-speed)", res.TimeOfImpact, tc.wantTOI)
			}
			if !vecEq(res.Normal, tc.wantNormal, absTol) {
				t.Fatalf("normal = %v, want %v (motion axis)", res.Normal, tc.wantNormal)
			}
			// Touch, not overlap: depth ~0, contact points coincide and lie
			// on the touching faces.
			if res.PenetrationDepth > absTol {
				t.Fatalf("depth at impact = %.3g, want ~0", res.PenetrationDepth)
			}
			if !approxEq(res.PointA.X, tc.wantTouchX, absTol) {
				t.Fatalf("contact point on A = %v, want x=%v face", res.PointA, tc.wantTouchX)
			}
			if !vecEq(res.PointA, res.PointB, absTol) {
				t.Fatalf("contact points differ: %v vs %v", res.PointA, res.PointB)
			}
			if res.Iterations <= 0 {
				t.Fatalf("iterations should be reported and positive")
			}
		})
	}
}

// TestSweep_RecedingReportsSafe is acceptance case 2: the same squares with
// the velocity reversed must report safe with the closest approach at t=0.
func TestSweep_RecedingReportsSafe(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(3, 0, 4, 1)
	cases := []struct {
		name       string
		velA, velB Vec2
	}{
		{"B recedes", Vec2{}, Vec2{X: 4}},
		{"both move apart", Vec2{X: -1}, Vec2{X: 3}},
		{"exactly parallel (tangential) motion", Vec2{}, Vec2{Y: 5}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := Sweep(a, b, tc.velA, tc.velB)
			if err != nil {
				t.Fatalf("sweep: %v", err)
			}
			if res.Verdict != VerdictSafe {
				t.Fatalf("verdict = %s, want safe", res.Verdict)
			}
			if res.TimeOfClosestApproach != 0 {
				t.Fatalf("closest approach t = %.15g, want exactly 0", res.TimeOfClosestApproach)
			}
			if !approxEq(res.MinDistance, 2, exactGapTol) {
				t.Fatalf("min distance = %.15g, want 2", res.MinDistance)
			}
			if !vecEq(res.Normal, Vec2{X: 1}, absTol) {
				t.Fatalf("normal = %v, want (1,0)", res.Normal)
			}
		})
	}
}

// TestSweep_CommonTranslationAndVelocityInvariance is acceptance case 3:
// applying a common rigid translation to both parts and/or a common extra
// velocity to both (relative motion unchanged) must leave the impact time
// and the contact normal unchanged.
func TestSweep_CommonTranslationAndVelocityInvariance(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 2, 3, 3) // corner-to-corner: gap sqrt(2), normal (1,1)/sqrt(2)
	velB := Vec2{X: -2, Y: -2}

	base, err := Sweep(a, b, Vec2{}, velB)
	if err != nil {
		t.Fatalf("base sweep: %v", err)
	}
	if base.Verdict != VerdictContact {
		t.Fatalf("base verdict = %s, want contact", base.Verdict)
	}
	// Closed form: closing speed is 2*sqrt(2) over a gap of sqrt(2).
	if !approxEq(base.TimeOfImpact, 0.5, exactGapTol) {
		t.Fatalf("base toi = %.15g, want 0.5", base.TimeOfImpact)
	}
	wantN := unitV(Vec2{X: 1, Y: 1})
	if !vecEq(base.Normal, wantN, absTol) {
		t.Fatalf("base normal = %v, want %v", base.Normal, wantN)
	}

	shifts := []Vec2{{X: 5, Y: 7}, {X: -13.25, Y: 4.5}, {X: 0.333, Y: 1.777}, {}}
	velShift := []Vec2{{X: 3, Y: -1}, {X: -7, Y: 11}, {X: 0.5, Y: 0.25}, {}}
	for _, sh := range shifts {
		for _, vs := range velShift {
			res, err := Sweep(translate(a, sh), translate(b, sh), vs, velB.Add(vs))
			if err != nil {
				t.Fatalf("shift %v velShift %v: %v", sh, vs, err)
			}
			if res.Verdict != VerdictContact {
				t.Fatalf("shift %v velShift %v: verdict = %s", sh, vs, res.Verdict)
			}
			if !approxEq(res.TimeOfImpact, base.TimeOfImpact, toiTol) {
				t.Fatalf("shift %v velShift %v: toi = %.15g, want %.15g",
					sh, vs, res.TimeOfImpact, base.TimeOfImpact)
			}
			if !vecEq(res.Normal, base.Normal, absTol) {
				t.Fatalf("shift %v velShift %v: normal = %v, want %v",
					sh, vs, res.Normal, base.Normal)
			}
			// The contact points track the common shift exactly:
			// p' = p + sh + vs*t*.
			drift := sh.Add(vs.Scale(base.TimeOfImpact))
			if !vecEq(res.PointA, base.PointA.Add(drift), absTol) {
				t.Fatalf("shift %v velShift %v: pointA = %v, want %v",
					sh, vs, res.PointA, base.PointA.Add(drift))
			}
		}
	}
}

// TestSweep_ZeroRelativeVelocityMatchesStatic is acceptance case 4: equal
// velocities on both parts make the sweep conclusion identical to one
// static evaluation of the initial pose.
func TestSweep_ZeroRelativeVelocityMatchesStatic(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(2.75, -0.5, 4.25, 1.5) // the known-gap example: 0.75
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("static: %v", err)
	}
	for _, v := range []Vec2{{X: 1.5, Y: -2}, {}, {X: -3, Y: 4}} {
		res, err := Sweep(a, b, v, v)
		if err != nil {
			t.Fatalf("v=%v: %v", v, err)
		}
		if res.Verdict != VerdictSafe {
			t.Fatalf("v=%v: verdict = %s, want safe", v, res.Verdict)
		}
		if res.TimeOfClosestApproach != 0 {
			t.Fatalf("v=%v: closest approach t = %g, want 0", v, res.TimeOfClosestApproach)
		}
		if !approxEq(res.MinDistance, static.Distance, exactGapTol) {
			t.Fatalf("v=%v: min distance = %.15g, want static %.15g", v, res.MinDistance, static.Distance)
		}
		if !vecEq(res.Normal, static.Normal, exactGapTol) {
			t.Fatalf("v=%v: normal = %v, want static %v", v, res.Normal, static.Normal)
		}
		if !vecEq(res.PointA, static.PointA, exactGapTol) || !vecEq(res.PointB, static.PointB, exactGapTol) {
			t.Fatalf("v=%v: closest points differ from static", v)
		}
	}

	// Overlapping pair with equal velocities: contact at t=0 with the
	// static penetration answer.
	a2 := rect(0, 0, 2, 1)
	b2 := rect(1.7, 0.1, 3.7, 1.9)
	st2, err := Evaluate(a2, b2)
	if err != nil {
		t.Fatalf("static overlap: %v", err)
	}
	res2, err := Sweep(a2, b2, Vec2{X: 1, Y: 1}, Vec2{X: 1, Y: 1})
	if err != nil {
		t.Fatalf("sweep overlap: %v", err)
	}
	if res2.Verdict != VerdictContact || res2.TimeOfImpact != 0 {
		t.Fatalf("overlap with equal velocities: verdict=%s toi=%g, want contact at 0",
			res2.Verdict, res2.TimeOfImpact)
	}
	if !approxEq(res2.PenetrationDepth, st2.PenetrationDepth, exactGapTol) {
		t.Fatalf("depth = %.15g, want static %.15g", res2.PenetrationDepth, st2.PenetrationDepth)
	}
	if !vecEq(res2.Normal, st2.Normal, exactGapTol) {
		t.Fatalf("normal = %v, want static %v", res2.Normal, st2.Normal)
	}
}

// TestSweep_InitialPenetration is acceptance case 5: parts already
// overlapping at t=0 report impact time 0 with the static penetration
// answer, regardless of the velocities.
func TestSweep_InitialPenetration(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(1.7, 0.1, 3.7, 1.9) // x overlap 0.3, y overlap 0.9 -> depth 0.3
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("static: %v", err)
	}
	for _, vb := range []Vec2{{X: 1}, {X: -1}, {}, {X: 0.3, Y: -0.7}} {
		res, err := Sweep(a, b, Vec2{}, vb)
		if err != nil {
			t.Fatalf("vb=%v: %v", vb, err)
		}
		if res.Verdict != VerdictContact {
			t.Fatalf("vb=%v: verdict = %s, want contact", vb, res.Verdict)
		}
		if res.TimeOfImpact != 0 {
			t.Fatalf("vb=%v: toi = %g, want exactly 0", vb, res.TimeOfImpact)
		}
		if !approxEq(res.PenetrationDepth, static.PenetrationDepth, exactGapTol) {
			t.Fatalf("vb=%v: depth = %.15g, want static %.15g", vb, res.PenetrationDepth, static.PenetrationDepth)
		}
		if !vecEq(res.Normal, static.Normal, exactGapTol) {
			t.Fatalf("vb=%v: normal = %v, want static %v", vb, res.Normal, static.Normal)
		}
		if !vecEq(res.PointA, static.PointA, exactGapTol) || !vecEq(res.PointB, static.PointB, exactGapTol) {
			t.Fatalf("vb=%v: contact points differ from static", vb)
		}
	}
}

// TestSweep_BriefContactCaughtNotSampled is acceptance case 6: a contact
// that exists only briefly inside the window. Uniformly slicing the window
// into 10 segments and running the static query per sample reports safe
// everywhere (verified below as the "forbidden approach"), while the
// continuous solve converges to the true first contact.
//
// Geometry: A = [0.66,0.91]x[0.41,0.462] (thin strip), B = [0,0.2]x[0,0.1]
// moving with v=(1,1). B's right edge reaches A's left edge at t=0.46 and
// B's bottom edge clears A's top edge at t=0.462: the contact interval is
// exactly [0.46, 0.462], which falls strictly between the 0.4 and 0.5
// samples of a 10-segment uniform sampling. (The coordinates are chosen so
// that no 0.1-multiple sample time lines up exactly with an edge — exact
// alignments are degenerate inputs for the static kernel.)
func TestSweep_BriefContactCaughtNotSampled(t *testing.T) {
	a := rect(0.66, 0.41, 0.91, 0.462)
	b := rect(0, 0, 0.2, 0.1)
	velB := Vec2{X: 1, Y: 1}

	// The forbidden discrete approach: 11 uniform static samples, all safe.
	for k := 0; k <= 10; k++ {
		ts := float64(k) / 10
		r, err := Evaluate(advance(a, Vec2{}, ts), advance(b, velB, ts))
		if err != nil {
			t.Fatalf("sample %v: %v", ts, err)
		}
		if r.Status != StatusSeparated {
			t.Fatalf("sample t=%v reports %s: the fixture must fool uniform sampling", ts, r.Status)
		}
	}
	// ... yet there is genuine penetration inside the window.
	mid, err := Evaluate(a, advance(b, velB, 0.461))
	if err != nil {
		t.Fatalf("mid-window evaluate: %v", err)
	}
	if mid.Status != StatusPenetrated {
		t.Fatalf("t=0.461 should be penetrated, got %s", mid.Status)
	}

	res, err := Sweep(a, b, Vec2{}, velB)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Verdict != VerdictContact {
		t.Fatalf("verdict = %s, want contact (sampling would have missed it)", res.Verdict)
	}
	if !approxEq(res.TimeOfImpact, 0.46, toiTol) {
		t.Fatalf("toi = %.15g, want 0.46", res.TimeOfImpact)
	}
	// At first contact B's right edge meets A's left edge: normal (-1,0),
	// contact points on the segment x=0.66, y in [0.46, 0.462].
	if !vecEq(res.Normal, Vec2{X: -1}, absTol) {
		t.Fatalf("normal = %v, want (-1,0)", res.Normal)
	}
	if !approxEq(res.PointA.X, 0.66, absTol) {
		t.Fatalf("contact x on A = %.15g, want 0.66", res.PointA.X)
	}
	if res.PointA.Y < 0.46-absTol || res.PointA.Y > 0.462+absTol {
		t.Fatalf("contact y on A = %.15g, want within [0.46, 0.462]", res.PointA.Y)
	}
	if !vecEq(res.PointA, res.PointB, absTol) {
		t.Fatalf("contact points differ: %v vs %v", res.PointA, res.PointB)
	}
}

// TestSweep_TouchExactlyAtWindowEnd is acceptance boundary 2a: the parts
// close the gap exactly at t=1. The impact time must come out (near) 1,
// not "safe".
func TestSweep_TouchExactlyAtWindowEnd(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1) // gap 1, closing speed 1: touch at t=1 exactly
	res, err := Sweep(a, b, Vec2{}, Vec2{X: -1})
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Verdict != VerdictContact {
		t.Fatalf("verdict = %s, want contact at the window end", res.Verdict)
	}
	if !approxEq(res.TimeOfImpact, 1, toiTol) {
		t.Fatalf("toi = %.15g, want 1", res.TimeOfImpact)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, absTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweep_NearMissAtWindowEnd is acceptance boundary 2b: the parts close
// almost the whole gap but remain separated at t=1. Must report safe with
// the closest approach at t=1.
func TestSweep_NearMissAtWindowEnd(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2.01, 0, 3.01, 1) // gap 1.01, closing speed 1: 0.01 short at t=1
	res, err := Sweep(a, b, Vec2{}, Vec2{X: -1})
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Verdict != VerdictSafe {
		t.Fatalf("verdict = %s, want safe (missed by 0.01)", res.Verdict)
	}
	if res.TimeOfClosestApproach != 1 {
		t.Fatalf("closest approach t = %.15g, want exactly 1", res.TimeOfClosestApproach)
	}
	wantGap := 2.01 - 1.0 - 1.0 // same float arithmetic as the kernel sees
	if !approxEq(res.MinDistance, wantGap, exactGapTol) {
		t.Fatalf("min distance = %.15g, want %.15g", res.MinDistance, wantGap)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, absTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweep_ClosestApproachInterior exercises the closest-approach solve on
// a trajectory whose minimum lies strictly inside the window, with a
// curved (vertex-region) gap function. With B = [1.4,1.6]x[2,3] moving at
// v = (1,-1.3), the gap is the distance from the point t*v to the Minkowski
// difference C = [-1.6,-0.4]x[-3,-1]:
//
//	g(t) = |t*v - c|  with closest corner c = (-0.4, -1)
//
// minimized at t* = (c·v)/(v·v) = 0.9/2.69 with g(t*) = |c×v|/|v| =
// 1.52/sqrt(2.69), and the gap normal is the unit vector along t*v - c.
func TestSweep_ClosestApproachInterior(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1.4, 2, 1.6, 3)
	velB := Vec2{X: 1, Y: -1.3}

	res, err := Sweep(a, b, Vec2{}, velB)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Verdict != VerdictSafe {
		t.Fatalf("verdict = %s, want safe", res.Verdict)
	}
	wantT := 0.9 / 2.69
	if !approxEq(res.TimeOfClosestApproach, wantT, toiTol) {
		t.Fatalf("closest approach t = %.15g, want %.15g", res.TimeOfClosestApproach, wantT)
	}
	wantGap := 1.52 / math.Sqrt(2.69)
	if !approxEq(res.MinDistance, wantGap, toiTol) {
		t.Fatalf("min distance = %.15g, want %.15g", res.MinDistance, wantGap)
	}
	// Gap normal at the minimum: unit vector along t*v - (-0.4,-1), which
	// is perpendicular to the relative velocity.
	wantN := unitV(Vec2{X: wantT + 0.4, Y: 1 - 1.3*wantT})
	if !vecEq(res.Normal, wantN, 1e-6) {
		t.Fatalf("normal = %v, want %v", res.Normal, wantN)
	}
	// The witness pair realizes the reported minimum gap.
	if d := res.PointA.Sub(res.PointB).Len(); !approxEq(d, res.MinDistance, absTol) {
		t.Fatalf("witness distance %.12g != reported min %.12g", d, res.MinDistance)
	}
}

// TestSweep_ValidationErrors: the sweep entry rejects the same illegal
// inputs as the static query (with the same codes), plus non-finite
// velocity components.
func TestSweep_ValidationErrors(t *testing.T) {
	valid := rect(0, 0, 1, 1)
	far := rect(5, 5, 6, 6)

	_, err := Sweep([]Vec2{{0, 0}, {1, 1}}, far, Vec2{}, Vec2{})
	assertCode(t, err, ErrTooFewVertices)

	_, err = Sweep([]Vec2{{0, 0}, {1, 1}, {2, 2}}, far, Vec2{}, Vec2{})
	assertCode(t, err, ErrDegeneratePolygon)

	_, err = Sweep([]Vec2{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}, far, Vec2{}, Vec2{})
	assertCode(t, err, ErrNonConvexPolygon)

	_, err = Sweep([]Vec2{{0, 0}, {1, 0}, {1, math.NaN()}}, far, Vec2{}, Vec2{})
	assertCode(t, err, ErrNonFiniteCoordinate)

	// Non-finite velocity components, either part, either axis.
	_, err = Sweep(valid, far, Vec2{X: math.NaN()}, Vec2{})
	assertCode(t, err, ErrNonFiniteCoordinate)
	_, err = Sweep(valid, far, Vec2{}, Vec2{Y: math.Inf(1)})
	assertCode(t, err, ErrNonFiniteCoordinate)
	_, err = Sweep(valid, far, Vec2{X: math.Inf(-1)}, Vec2{})
	assertCode(t, err, ErrNonFiniteCoordinate)

	// The velocity error message must say so (not blame a vertex).
	_, err = Sweep(valid, far, Vec2{X: math.NaN()}, Vec2{})
	if ke, ok := err.(*KernelError); !ok || !strings.Contains(ke.Message, "velocity") {
		t.Fatalf("velocity error should mention the velocity: %v", err)
	}
}

// TestSweep_BudgetExhausted proves the sweep iteration caps are hard errors
// (SWEEP_NO_CONVERGENCE), never an infinite loop and never a fake "safe".
func TestSweep_BudgetExhausted(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(3, 0, 4, 1)

	origSteps := sweepMaxSteps
	sweepMaxSteps = 0
	_, err := Sweep(a, b, Vec2{}, Vec2{X: -4})
	sweepMaxSteps = origSteps
	assertCode(t, err, ErrSweepNoConvergence)

	// The closest-approach bisection has its own budget.
	origBisect := sweepBisectSteps
	sweepBisectSteps = 0
	_, err = Sweep(rect(0, 0, 1, 1), rect(1.4, 2, 1.6, 3), Vec2{}, Vec2{X: 1, Y: -1.3})
	sweepBisectSteps = origBisect
	assertCode(t, err, ErrSweepNoConvergence)
}

// TestSweep_IterationsBounded: the reported evaluation count is positive
// and within the declared budgets across a spread of scenarios.
func TestSweep_IterationsBounded(t *testing.T) {
	a := rect(0, 0, 1, 1)
	scenarios := []struct {
		b          []Vec2
		velA, velB Vec2
	}{
		{rect(3, 0, 4, 1), Vec2{}, Vec2{X: -4}},                   // head-on impact
		{rect(3, 0, 4, 1), Vec2{}, Vec2{X: 4}},                    // receding
		{rect(1.4, 2, 1.6, 3), Vec2{}, Vec2{X: 1, Y: -1.3}},       // interior minimum
		{rect(0.66, 0.41, 0.91, 0.462), Vec2{}, Vec2{X: 1, Y: 1}}, // brief contact
		{rect(2, 0, 3, 1), Vec2{}, Vec2{X: -1}},                   // touch at t=1
		{rect(2.01, 0, 3.01, 1), Vec2{}, Vec2{X: -1}},             // near miss at t=1
		{rect(1.7, 0.1, 3.7, 1.9), Vec2{}, Vec2{X: 1}},            // initial penetration
	}
	for i, sc := range scenarios {
		res, err := Sweep(a, sc.b, sc.velA, sc.velB)
		if err != nil {
			t.Fatalf("scenario %d: %v", i, err)
		}
		if res.Iterations <= 0 || res.Iterations > MaxSweepSteps+MaxSweepBisectSteps+4 {
			t.Fatalf("scenario %d: iterations %d out of bounds", i, res.Iterations)
		}
	}
}
