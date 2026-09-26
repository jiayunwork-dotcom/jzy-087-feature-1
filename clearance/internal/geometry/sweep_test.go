package geometry

import (
	"math"
	"math/rand"
	"testing"
)

// Tolerances for the sweep assertions, stated explicitly as elsewhere:
//   - toiTol: 1e-9 absolute, for contact/closest-approach times compared
//     against closed-form values (gap / closing rate);
//   - exactGapTol / absTol / relTol (validation_test.go) for gaps, normals
//     and witness points.
const toiTol = 1e-9

// sweepHeadOn builds the acceptance fixture: axis-aligned unit squares with
// A = [0,1]x[0,1] at rest and B = [1+gap, 2+gap]x[0,1] moving along -x at
// the given speed. The closed-form time of impact is gap/speed and the
// contact normal is +X (from A's right edge toward B's left edge).
func sweepHeadOn(gap, speed float64) (a, b []Vec2, va, vb Vec2) {
	return rect(0, 0, 1, 1), rect(1+gap, 0, 2+gap, 1), Vec2{}, Vec2{X: -speed}
}

// TestSweepHeadOnClosedForm pins the first-contact time to the analytic
// value gap/closing-rate and the contact normal to the motion axis.
func TestSweepHeadOnClosedForm(t *testing.T) {
	cases := []struct {
		name    string
		gap     float64
		speed   float64
		wantTOI float64
	}{
		{"binary exact", 1.5, 2, 0.75},
		{"non dyadic", 0.3, 1.1, 0.3 / 1.1},
		{"two thirds", 2, 3, 2.0 / 3.0},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a, b, va, vb := sweepHeadOn(tc.gap, tc.speed)
			res, err := Sweep(a, b, va, vb)
			if err != nil {
				t.Fatalf("sweep: %v", err)
			}
			if res.Status != StatusImpact {
				t.Fatalf("status = %s, want impact", res.Status)
			}
			if !approxEq(res.Time, tc.wantTOI, toiTol) {
				t.Fatalf("time of impact = %.15g, want %.15g (gap/speed)", res.Time, tc.wantTOI)
			}
			if !vecEq(res.Normal, Vec2{X: 1}, exactGapTol) {
				t.Fatalf("normal = %v, want (1,0)", res.Normal)
			}
			if res.Distance != 0 || res.PenetrationDepth != 0 {
				t.Fatalf("impact must report zero gap/depth, got %v / %v",
					res.Distance, res.PenetrationDepth)
			}
			// The contact points sit on the touching edges x=1 and coincide.
			if !approxEq(res.PointA.X, 1, absTol) {
				t.Fatalf("contact point on A = %v, expected the x=1 edge", res.PointA)
			}
			if !approxEq(res.PointB.X, 1, absTol) {
				t.Fatalf("contact point on B = %v, expected its left edge at x=1", res.PointB)
			}
			if d := res.PointA.Sub(res.PointB).Len(); d > absTol {
				t.Fatalf("contact points do not coincide: |pA-pB| = %.3g", d)
			}
			if res.Iterations <= 0 || res.Iterations > MaxSweepIterations {
				t.Fatalf("iterations out of bounds: %d", res.Iterations)
			}
		})
	}
}

// TestSweepRecedingIsClear: motion pointing away must report the whole
// window safe, with the closest approach exactly at t=0.
func TestSweepRecedingIsClear(t *testing.T) {
	a, b, _, _ := sweepHeadOn(1.5, 2)
	res, err := Sweep(a, b, Vec2{}, Vec2{X: 2})
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusClear {
		t.Fatalf("status = %s, want clear", res.Status)
	}
	if res.Time != 0 {
		t.Fatalf("closest-approach time = %.15g, want exactly 0 for receding motion", res.Time)
	}
	if !approxEq(res.Distance, 1.5, exactGapTol) {
		t.Fatalf("min gap = %.15g, want 1.5", res.Distance)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, exactGapTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweepRigidShiftInvariance: applying one common rigid translation to
// both parts and one common velocity boost to both (relative motion
// unchanged) must leave the time of impact and the contact normal
// unchanged; contact points shift by T + u*t.
func TestSweepRigidShiftInvariance(t *testing.T) {
	a, b, va, vb := sweepHeadOn(1.5, 2)
	base, err := Sweep(a, b, va, vb)
	if err != nil {
		t.Fatalf("base sweep: %v", err)
	}
	if base.Status != StatusImpact {
		t.Fatalf("base status = %s, want impact", base.Status)
	}

	shift := Vec2{X: 37.25, Y: -11.5}
	boost := Vec2{X: 3.5, Y: 4.25}
	got, err := Sweep(translate(a, shift), translate(b, shift), va.Add(boost), vb.Add(boost))
	if err != nil {
		t.Fatalf("shifted sweep: %v", err)
	}
	if got.Status != StatusImpact {
		t.Fatalf("shifted status = %s, want impact", got.Status)
	}
	if !approxEq(got.Time, base.Time, toiTol) {
		t.Fatalf("time of impact changed under rigid shift: %.15g vs %.15g", got.Time, base.Time)
	}
	if !vecEq(got.Normal, base.Normal, absTol) {
		t.Fatalf("normal changed under rigid shift: %v vs %v", got.Normal, base.Normal)
	}
	// Contact points ride along with the shift and the boost.
	wantPA := base.PointA.Add(shift).Add(boost.Scale(base.Time))
	wantPB := base.PointB.Add(shift).Add(boost.Scale(base.Time))
	if !approxEqRel(got.PointA.X, wantPA.X, relTol, absTol) ||
		!approxEqRel(got.PointA.Y, wantPA.Y, relTol, absTol) {
		t.Fatalf("contact point A = %v, want %v", got.PointA, wantPA)
	}
	if !approxEqRel(got.PointB.X, wantPB.X, relTol, absTol) ||
		!approxEqRel(got.PointB.Y, wantPB.Y, relTol, absTol) {
		t.Fatalf("contact point B = %v, want %v", got.PointB, wantPB)
	}
}

// TestSweepZeroRelativeVelocityMatchesStatic: equal velocities (including
// both zero) freeze the relative configuration, so the sweep verdict must
// coincide with the static verdict at the initial pose.
func TestSweepZeroRelativeVelocityMatchesStatic(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(2.75, -0.5, 4.25, 1.5) // known gap 0.75, normal +X
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("static evaluate: %v", err)
	}
	for _, v := range []Vec2{{}, {X: 0.4, Y: -0.3}, {X: -2, Y: 7}} {
		res, err := Sweep(a, b, v, v)
		if err != nil {
			t.Fatalf("sweep v=%v: %v", v, err)
		}
		if res.Status != StatusClear {
			t.Fatalf("v=%v: status = %s, want clear", v, res.Status)
		}
		if res.Time != 0 {
			t.Fatalf("v=%v: closest time = %.15g, want exactly 0", v, res.Time)
		}
		if !approxEq(res.Distance, static.Distance, exactGapTol) {
			t.Fatalf("v=%v: distance = %.15g, static = %.15g", v, res.Distance, static.Distance)
		}
		if !vecEq(res.Normal, static.Normal, exactGapTol) {
			t.Fatalf("v=%v: normal = %v, static = %v", v, res.Normal, static.Normal)
		}
		if !vecEq(res.PointA, static.PointA, exactGapTol) || !vecEq(res.PointB, static.PointB, exactGapTol) {
			t.Fatalf("v=%v: closest points %v/%v, static = %v/%v",
				v, res.PointA, res.PointB, static.PointA, static.PointB)
		}
	}

	// Penetrating pair with zero relative velocity: impact at t=0 with the
	// static penetration verdict.
	a2 := rect(0, 0, 2, 1)
	b2 := rect(1.7, 0.1, 3.7, 1.9)
	static2, err := Evaluate(a2, b2)
	if err != nil {
		t.Fatalf("static evaluate: %v", err)
	}
	res2, err := Sweep(a2, b2, Vec2{X: 1, Y: 1}, Vec2{X: 1, Y: 1})
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res2.Status != StatusImpact || res2.Time != 0 {
		t.Fatalf("status/time = %s/%.15g, want impact at 0", res2.Status, res2.Time)
	}
	if !approxEq(res2.PenetrationDepth, static2.PenetrationDepth, exactGapTol) {
		t.Fatalf("depth = %.15g, static = %.15g", res2.PenetrationDepth, static2.PenetrationDepth)
	}
	if !vecEq(res2.Normal, static2.Normal, exactGapTol) {
		t.Fatalf("normal = %v, static = %v", res2.Normal, static2.Normal)
	}
}

// TestSweepInitialPenetration: overlapping at t=0 means the first-contact
// time is 0 and the contact information is the static penetration verdict,
// regardless of the velocities.
func TestSweepInitialPenetration(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(1.7, 0.1, 3.7, 1.9) // x overlap 0.3, y overlap 0.9 -> depth 0.3
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("static evaluate: %v", err)
	}
	res, err := Sweep(a, b, Vec2{X: 5, Y: 5}, Vec2{X: -9, Y: 2})
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusImpact {
		t.Fatalf("status = %s, want impact", res.Status)
	}
	if res.Time != 0 {
		t.Fatalf("time of impact = %.15g, want exactly 0 for initial penetration", res.Time)
	}
	if !approxEq(res.PenetrationDepth, 0.3, exactGapTol) {
		t.Fatalf("depth = %.15g, want 0.3", res.PenetrationDepth)
	}
	if !vecEq(res.Normal, static.Normal, exactGapTol) {
		t.Fatalf("normal = %v, static = %v", res.Normal, static.Normal)
	}
	if !vecEq(res.PointA, static.PointA, exactGapTol) || !vecEq(res.PointB, static.PointB, exactGapTol) {
		t.Fatalf("contact points %v/%v, static = %v/%v", res.PointA, res.PointB, static.PointA, static.PointB)
	}
}

// TestSweepGrazingContactCaught is the anti-sampling witness: the overlap
// window is t in (0.4, 0.45), which contains no sample of a uniform 8-segment
// grid, so the coarse "slice the window and run static checks" approach
// reports the whole window safe. The continuous solver must catch the
// contact and pin it to t=0.4.
func TestSweepGrazingContactCaught(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := translate(rect(0, 0, 1, 1), Vec2{X: -1.8, Y: 8})
	vb := Vec2{X: 2, Y: -20}
	// Overlap needs B.x in (-1,1) (t > 0.4) and B.y in (-1,1)
	// (0.35 < t < 0.45): contact during t in (0.4, 0.45), first contact at
	// t = 0.4 when B's right edge reaches x=0 with y-overlap [0.9, 1].

	// The forbidden approximation: 8 uniform slices, 9 static checks.
	for k := 0; k <= 8; k++ {
		ts := float64(k) / 8
		frame, err := Evaluate(a, translate(b, vb.Scale(ts)))
		if err != nil {
			t.Fatalf("static frame at t=%v: %v", ts, err)
		}
		if frame.Status != StatusSeparated {
			t.Fatalf("fixture broken: coarse sampling at t=%v detects %s", ts, frame.Status)
		}
	}
	// ... so a sample-based implementation would declare the window safe.

	res, err := Sweep(a, b, Vec2{}, vb)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusImpact {
		t.Fatalf("status = %s, want impact (grazing contact missed!)", res.Status)
	}
	if !approxEq(res.Time, 0.4, toiTol) {
		t.Fatalf("time of impact = %.15g, want 0.4", res.Time)
	}
	// B approaches from the -x side: the contact normal points from A to B.
	if !vecEq(res.Normal, Vec2{X: -1}, absTol) {
		t.Fatalf("normal = %v, want (-1,0)", res.Normal)
	}
	if !approxEq(res.PointA.X, 0, absTol) || !approxEq(res.PointB.X, 0, absTol) {
		t.Fatalf("contact points = %v/%v, expected both on the touching line x=0",
			res.PointA, res.PointB)
	}
	if d := res.PointA.Sub(res.PointB).Len(); d > absTol {
		t.Fatalf("contact points do not coincide: |pA-pB| = %.3g", d)
	}
	if res.Iterations <= 0 || res.Iterations > MaxSweepIterations {
		t.Fatalf("iterations out of bounds: %d", res.Iterations)
	}
}

// TestSweepTouchExactlyAtWindowEnd: the closing distance is covered exactly
// at t=1; the verdict must be impact with a time close to 1.
func TestSweepTouchExactlyAtWindowEnd(t *testing.T) {
	a, b, va, vb := sweepHeadOn(1.5, 1.5) // gap 1.5, closing 1.5 -> touch at t=1
	res, err := Sweep(a, b, va, vb)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusImpact {
		t.Fatalf("status = %s, want impact at the window end", res.Status)
	}
	if !approxEq(res.Time, 1, toiTol) {
		t.Fatalf("time of impact = %.15g, want ~1", res.Time)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, absTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweepShortAtWindowEnd: the parts approach but are still 0.1 apart at
// t=1; the verdict must be clear with the closest approach exactly at t=1.
func TestSweepShortAtWindowEnd(t *testing.T) {
	a, b, va, vb := sweepHeadOn(1.5, 1.4) // closes 1.4 < 1.5
	res, err := Sweep(a, b, va, vb)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusClear {
		t.Fatalf("status = %s, want clear", res.Status)
	}
	if res.Time != 1 {
		t.Fatalf("closest-approach time = %.15g, want exactly 1", res.Time)
	}
	if !approxEq(res.Distance, 0.1, exactGapTol) {
		t.Fatalf("min gap = %.15g, want 0.1", res.Distance)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, exactGapTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweepPassByInteriorMinimum: B passes below A; the gap minimum is
// interior to the window. At t: B = [2-t, 3-t]x[-2t, 1-2t]; for t in
// (0.5, 1) the gap is sqrt((1-t)^2 + (2t-1)^2), minimized at t=0.6 with
// value sqrt(0.2) between the corners (1,0) and (1.4,-0.2).
func TestSweepPassByInteriorMinimum(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1)
	vb := Vec2{X: -1, Y: -2}
	res, err := Sweep(a, b, Vec2{}, vb)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusClear {
		t.Fatalf("status = %s, want clear", res.Status)
	}
	if !approxEq(res.Time, 0.6, toiTol) {
		t.Fatalf("closest-approach time = %.15g, want 0.6", res.Time)
	}
	if !approxEq(res.Distance, math.Sqrt(0.2), toiTol) {
		t.Fatalf("min gap = %.15g, want sqrt(0.2) = %.15g", res.Distance, math.Sqrt(0.2))
	}
	wantN := Vec2{X: 0.4, Y: -0.2}.Normalized()
	if !vecEq(res.Normal, wantN, absTol) {
		t.Fatalf("normal = %v, want %v", res.Normal, wantN)
	}
}

// TestSweepObliqueCrossCheck validates the reported time of impact against
// an independent oracle: a bisection on the static penetration predicate
// (itself a convergent method, not a fixed sample grid).
func TestSweepObliqueCrossCheck(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(3, 0.2, 4, 1.2)
	vb := Vec2{X: -3, Y: -0.4}
	// The x gap of 2 closes at t = 2/3; y overlap persists through the
	// window, so the parts stay penetrated from 2/3 on.

	res, err := Sweep(a, b, Vec2{}, vb)
	if err != nil {
		t.Fatalf("sweep: %v", err)
	}
	if res.Status != StatusImpact {
		t.Fatalf("status = %s, want impact", res.Status)
	}
	if !approxEq(res.Time, 2.0/3.0, toiTol) {
		t.Fatalf("time of impact = %.15g, want 2/3", res.Time)
	}

	lo, hi := 0.0, 1.0
	for i := 0; i < 100; i++ {
		mid := lo + (hi-lo)/2
		frame, err := Evaluate(a, translate(b, vb.Scale(mid)))
		if err != nil {
			t.Fatalf("oracle frame: %v", err)
		}
		if frame.Status == StatusPenetrated {
			hi = mid
		} else {
			lo = mid
		}
	}
	oracle := lo + (hi-lo)/2
	if !approxEq(res.Time, oracle, 1e-8) {
		t.Fatalf("sweep time %.15g disagrees with predicate-bisection oracle %.15g", res.Time, oracle)
	}
}

// TestSweepBudgetExhausted proves the advancement step cap is a hard error,
// not an infinite loop and not a fake "safe" answer.
func TestSweepBudgetExhausted(t *testing.T) {
	a, b, va, vb := sweepHeadOn(1.5, 2)
	orig := sweepMaxSteps
	sweepMaxSteps = 0
	_, err := Sweep(a, b, va, vb)
	sweepMaxSteps = orig
	assertCode(t, err, ErrSweepNoConvergence)
}

// TestSweepValidation rejects the same malformed geometry as the static
// entry point, plus non-finite velocities.
func TestSweepValidation(t *testing.T) {
	okA := rect(0, 0, 1, 1)
	okB := rect(2, 0, 3, 1)
	zero := Vec2{}

	t.Run("too few vertices", func(t *testing.T) {
		_, err := Sweep([]Vec2{{0, 0}, {1, 1}}, okB, zero, zero)
		assertCode(t, err, ErrTooFewVertices)
	})
	t.Run("degenerate polygon", func(t *testing.T) {
		_, err := Sweep(tri(Vec2{}, Vec2{X: 1, Y: 1}, Vec2{X: 2, Y: 2}), okB, zero, zero)
		assertCode(t, err, ErrDegeneratePolygon)
	})
	t.Run("non convex polygon", func(t *testing.T) {
		lshape := []Vec2{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}
		_, err := Sweep(lshape, okB, zero, zero)
		assertCode(t, err, ErrNonConvexPolygon)
	})
	t.Run("non finite vertex", func(t *testing.T) {
		bad := []Vec2{{math.NaN(), 0}, {1, 0}, {1, 1}}
		_, err := Sweep(bad, okB, zero, zero)
		assertCode(t, err, ErrNonFiniteCoordinate)
	})
	t.Run("non finite velocity A", func(t *testing.T) {
		_, err := Sweep(okA, okB, Vec2{X: math.Inf(1)}, zero)
		assertCode(t, err, ErrNonFiniteCoordinate)
	})
	t.Run("non finite velocity B", func(t *testing.T) {
		_, err := Sweep(okA, okB, zero, Vec2{Y: math.NaN()})
		assertCode(t, err, ErrNonFiniteCoordinate)
	})
	t.Run("relative velocity overflows", func(t *testing.T) {
		// Each velocity is finite, but their difference is not.
		_, err := Sweep(okA, okB, Vec2{X: -1.7e308}, Vec2{X: 1.7e308})
		assertCode(t, err, ErrNonFiniteCoordinate)
	})
}

// TestSweepRandomizedCrossCheck sweeps random convex polygons under random
// constant velocities and cross-checks the verdicts:
//   - impact: the reported time must match a bisection oracle on the static
//     penetration predicate (a convergent reference, not a sample grid);
//   - clear: a dense grid of static frames must confirm no contact.
func TestSweepRandomizedCrossCheck(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	impacts, clears := 0, 0
	for iter := 0; iter < 400 && (impacts < 60 || clears < 60); iter++ {
		a := randomConvex(rng, 3+rng.Intn(4), 1.5)
		theta := rng.Float64() * 2 * math.Pi
		dir := Vec2{X: math.Cos(theta), Y: math.Sin(theta)}
		startDist := 4.5 + 3*rng.Float64() // radii sum to 2.5: always separated at t=0
		b := translate(randomConvex(rng, 3+rng.Intn(4), 1.0), dir.Scale(startDist))
		// Velocity: roughly toward A with a random tangential component.
		speed := startDist + 1 + 4*rng.Float64()
		vb := dir.Scale(-speed).Add(Vec2{X: rng.Float64() - 0.5, Y: rng.Float64() - 0.5}.Scale(3))

		res, err := Sweep(a, b, Vec2{}, vb)
		if err != nil {
			t.Fatalf("iter %d: sweep: %v", iter, err)
		}
		if res.Time < 0 || res.Time > 1 {
			t.Fatalf("iter %d: time %.15g outside [0,1]", iter, res.Time)
		}

		switch res.Status {
		case StatusImpact:
			// The oracle needs a penetrated frame at t=1 to bracket against.
			f1, err := Evaluate(a, translate(b, vb))
			if err != nil {
				t.Fatalf("iter %d: oracle frame: %v", iter, err)
			}
			if f1.Status != StatusPenetrated {
				continue // touch-and-separate: no monotone oracle; skip
			}
			lo, hi := 0.0, 1.0
			for i := 0; i < 80; i++ {
				mid := lo + (hi-lo)/2
				frame, err := Evaluate(a, translate(b, vb.Scale(mid)))
				if err != nil {
					t.Fatalf("iter %d: oracle frame: %v", iter, err)
				}
				if frame.Status == StatusPenetrated {
					hi = mid
				} else {
					lo = mid
				}
			}
			oracle := lo + (hi-lo)/2
			if !approxEq(res.Time, oracle, 1e-6) {
				t.Fatalf("iter %d: sweep TOI %.15g, oracle %.15g", iter, res.Time, oracle)
			}
			impacts++
		case StatusClear:
			// Dense static grid must confirm the window is contact-free,
			// and the reported minimum gap must not undershoot the grid.
			gridMin := math.Inf(1)
			for k := 0; k <= 100; k++ {
				ts := float64(k) / 100
				frame, err := Evaluate(a, translate(b, vb.Scale(ts)))
				if err != nil {
					t.Fatalf("iter %d: grid frame: %v", iter, err)
				}
				if frame.Status != StatusSeparated {
					t.Fatalf("iter %d: sweep says clear but t=%.2f is %s", iter, ts, frame.Status)
				}
				gridMin = math.Min(gridMin, frame.Distance)
			}
			if gridMin < res.Distance-1e-6 {
				t.Fatalf("iter %d: reported min gap %.15g exceeds grid minimum %.15g",
					iter, res.Distance, gridMin)
			}
			clears++
		}
	}
	if impacts == 0 || clears == 0 {
		t.Fatalf("randomized check degenerate: impacts=%d clears=%d", impacts, clears)
	}
}
