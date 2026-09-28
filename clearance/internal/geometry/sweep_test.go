package geometry

import (
	"math"
	"math/rand"
	"testing"
)

// Swept-query tolerances. Like the static tests they are stated up front:
//   - sweepExactTol: analytically known impact times / normals (1e-9);
//   - sweepPointTol: contact and witness point positions;
//   - sweepGridN:    grid sizes used to prove uniform sampling misses the
//     grazing contact.
const (
	sweepExactTol  = 1e-9
	sweepPointTol  = 1e-8
	sweepTimeTolTc = 1e-11
)

// sweptRectHeadOn is the analytic first-acceptance fixture: A is the unit
// square [0,1]x[0,1] at rest; B is the unit square [2,3]x[0,1] moving with
// velocity (-u,0). The initial edge-to-edge gap is exactly 1, the closing
// speed is u, so the time of first impact is 1/u with normal +X.
func sweptHeadOn(u float64) SweptInput {
	return SweptInput{
		A:         rect(0, 0, 1, 1),
		B:         rect(2, 0, 3, 1),
		VelocityA: Vec2{},
		VelocityB: Vec2{X: -u, Y: 0},
	}
}

// TestSweep_HeadOn_ClosedFormTime: the impact time must equal the closed form
// gap/closing-rate, and the contact normal must align with the motion axis.
// Closing speeds are chosen so the analytic impact time 1/u lies within the
// window (u >= 1); slower approaches never touch in the window and are
// covered separately by the near-miss / end-touch tests.
func TestSweep_HeadOn_ClosedFormTime(t *testing.T) {
	for _, u := range []float64{2.0, 3.5, 1.0, 10.0} {
		in := sweptHeadOn(u)
		res, err := Swept(in)
		if err != nil {
			t.Fatalf("u=%v: %v", u, err)
		}
		wantT := 1.0 / u
		if res.Status != SweptStatusContact {
			t.Fatalf("u=%v status = %s, want contact", u, res.Status)
		}
		if !approxEq(res.Time, wantT, sweepExactTol) {
			t.Fatalf("u=%v time = %.15g, want %.15g", u, res.Time, wantT)
		}
		if !vecEq(res.Normal, Vec2{X: 1}, sweepExactTol) {
			t.Fatalf("u=%v normal = %v, want (1,0)", u, res.Normal)
		}
		if res.Distance != 0 {
			t.Fatalf("u=%v contact distance = %v, want 0", u, res.Distance)
		}
		if !vecEq(res.PointA, res.PointB, sweepPointTol) {
			t.Fatalf("u=%v contact witnesses do not coincide: %v vs %v",
				u, res.PointA, res.PointB)
		}
		// The contact point lies on A's facing edge x=1.
		if !approxEq(res.PointA.X, 1.0, sweepPointTol) {
			t.Fatalf("u=%v contact x = %.12g, want 1", u, res.PointA.X)
		}
		// And it is inside the [0,1] vertical overlap band.
		if res.PointA.Y < -sweepPointTol || res.PointA.Y > 1+sweepPointTol {
			t.Fatalf("u=%v contact y = %.12g outside the edge band", u, res.PointA.Y)
		}
		if res.Time < 0 || res.Time > 1 {
			t.Fatalf("impact time %.15g outside [0,1]", res.Time)
		}
		if res.Iterations <= 0 || res.Iterations > MaxSweepIterations {
			t.Fatalf("iterations out of bounds: %d", res.Iterations)
		}
	}
}

// TestSweep_Receding_IsSafeAtTimeZero: with the approach velocity reversed
// the parts only move apart; the whole window is safe and the closest instant
// is exactly t=0.
func TestSweep_Receding_IsSafeAtTimeZero(t *testing.T) {
	for _, v := range []Vec2{{X: 2}, {X: 0.5}, {X: 3, Y: 1}, {X: -0.0, Y: 4}} {
		in := SweptInput{
			A:         rect(0, 0, 1, 1),
			B:         rect(2, 0, 3, 1),
			VelocityA: Vec2{},
			VelocityB: v,
		}
		res, err := Swept(in)
		if err != nil {
			t.Fatalf("v=%v: %v", v, err)
		}
		if res.Status != SweptStatusSafe {
			t.Fatalf("v=%v status = %s, want safe", v, res.Status)
		}
		if res.Time != 0 {
			t.Fatalf("v=%v closest time = %.15g, want 0", v, res.Time)
		}
		if !approxEq(res.Distance, 1.0, sweepExactTol) {
			t.Fatalf("v=%v min gap = %.15g, want 1", v, res.Distance)
		}
		if !vecEq(res.Normal, Vec2{X: 1}, sweepExactTol) {
			t.Fatalf("v=%v normal = %v, want (1,0)", v, res.Normal)
		}
	}
}

// TestSweep_GalileanInvariance: applying the same extra rigid translation to
// both initial poses and the same extra velocity to both parts leaves the
// relative motion unchanged; the first impact time and contact normal must
// remain exactly the same (world contact points translate by off + t*vc).
func TestSweep_GalileanInvariance(t *testing.T) {
	base := sweptHeadOn(2.0) // impact at t=0.5
	shifts := []struct {
		off Vec2
		vc  Vec2
	}{
		{Vec2{}, Vec2{X: 5, Y: -2}},
		{Vec2{X: -7, Y: 3.25}, Vec2{}},
		{Vec2{X: 100, Y: -200}, Vec2{X: 1.5, Y: 2.5}},
		{Vec2{X: 0.333, Y: 1.777}, Vec2{X: -9.25, Y: -0.125}},
	}
	want, err := Swept(base)
	if err != nil {
		t.Fatal(err)
	}
	for _, sh := range shifts {
		moved := SweptInput{
			A:         translate(base.A, sh.off),
			B:         translate(base.B, sh.off),
			VelocityA: base.VelocityA.Add(sh.vc),
			VelocityB: base.VelocityB.Add(sh.vc),
		}
		got, err := Swept(moved)
		if err != nil {
			t.Fatalf("shift %+v: %v", sh, err)
		}
		if got.Status != want.Status {
			t.Fatalf("shift %+v: status %s != %s", sh, got.Status, want.Status)
		}
		if !approxEq(got.Time, want.Time, sweepExactTol) {
			t.Fatalf("shift %+v: time %.15g != %.15g", sh, got.Time, want.Time)
		}
		if !vecEq(got.Normal, want.Normal, sweepExactTol) {
			t.Fatalf("shift %+v: normal %v != %v", sh, got.Normal, want.Normal)
		}
		if !vecEq(got.RelativeVelocity, want.RelativeVelocity, 1e-12) {
			t.Fatalf("shift %+v: relative velocity changed: %v vs %v",
				sh, got.RelativeVelocity, want.RelativeVelocity)
		}
		// World contact points translate by off + t*vc. Coordinates as large
		// as ~300 carry a correspondingly larger absolute round-off, so the
		// witness comparison uses a scale-aware tolerance.
		shift := sh.off.Add(sh.vc.Scale(want.Time))
		scale := 1 + math.Max(math.Abs(want.PointA.Add(shift).X),
			math.Abs(want.PointA.Add(shift).Y))
		pointTol := 1e-10 * scale
		if !vecEq(got.PointA, want.PointA.Add(shift), pointTol) {
			t.Fatalf("shift %+v: contact point %v, want %v (tol %.2g)",
				sh, got.PointA, want.PointA.Add(shift), pointTol)
		}
	}
}

// TestSweep_ZeroRelativeVelocity_MatchesStatic: equal velocities (common
// translation or both stationary) freeze the relative geometry, so the swept
// answer must agree field-for-field with a single static evaluation of the
// initial poses — both in the separated and penetrated branches.
func TestSweep_ZeroRelativeVelocity_MatchesStatic(t *testing.T) {
	cases := []struct {
		name       string
		a, b       []Vec2
		off        Vec2
		penetrated bool
	}{
		{"separated rectangles", rect(0, 0, 2, 1), rect(2.75, -0.5, 4.25, 1.5), Vec2{}, false},
		{"separated with common offset", rect(0, 0, 1, 1), rect(2, 0, 3, 1), Vec2{X: 4, Y: -6}, false},
		{"penetrated rectangles", rect(0, 0, 2, 1), rect(1.7, 0.1, 3.7, 1.9), Vec2{}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			b := translate(tc.b, tc.off)
			static, err := Evaluate(tc.a, b)
			if err != nil {
				t.Fatal(err)
			}
			for _, common := range []Vec2{{}, {X: 3, Y: -1}, {X: 100, Y: 0.25}} {
				res, err := Swept(SweptInput{
					A: tc.a, B: tc.b, OffsetB: tc.off,
					VelocityA: common, VelocityB: common,
				})
				if err != nil {
					t.Fatalf("common v=%v: %v", common, err)
				}
				if static.Status == StatusSeparated {
					if res.Status != SweptStatusSafe {
						t.Fatalf("common v=%v: status = %s, want safe", common, res.Status)
					}
				} else {
					if res.Status != SweptStatusContact {
						t.Fatalf("common v=%v: status = %s, want contact", common, res.Status)
					}
				}
				if res.Time != 0 {
					t.Fatalf("common v=%v: time = %v, want 0", common, res.Time)
				}
				if !approxEqRel(res.Distance, static.Distance, relTol, absTol) {
					t.Fatalf("common v=%v: distance %.12g != static %.12g",
						common, res.Distance, static.Distance)
				}
				if !approxEqRel(res.PenetrationDepth, static.PenetrationDepth, relTol, absTol) {
					t.Fatalf("common v=%v: depth %.12g != static %.12g",
						common, res.PenetrationDepth, static.PenetrationDepth)
				}
				if !vecEq(res.Normal, static.Normal, absTol) {
					t.Fatalf("common v=%v: normal %v != static %v",
						common, res.Normal, static.Normal)
				}
				if !vecEq(res.PointA, static.PointA, absTol) {
					t.Fatalf("common v=%v: pointA %v != static %v",
						common, res.PointA, static.PointA)
				}
				if !vecEq(res.PointB, static.PointB, absTol) {
					t.Fatalf("common v=%v: pointB %v != static %v",
						common, res.PointB, static.PointB)
				}
			}
		})
	}
}

// TestSweep_InitialPenetration: overlap present at t=0 yields impact time 0
// with the contact information taken verbatim from the static penetration
// result (including a non-zero depth), regardless of subsequent motion.
func TestSweep_InitialPenetration(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(1.7, 0.1, 3.7, 1.9)
	static, err := Evaluate(a, b)
	if err != nil {
		t.Fatal(err)
	}
	for _, v := range []Vec2{{}, {X: 5}, {X: -5, Y: 2}} {
		res, err := Swept(SweptInput{A: a, B: b, VelocityB: v})
		if err != nil {
			t.Fatalf("v=%v: %v", v, err)
		}
		if res.Status != SweptStatusContact || res.Time != 0 {
			t.Fatalf("v=%v: status=%s time=%v, want contact at 0", v, res.Status, res.Time)
		}
		if !approxEq(res.PenetrationDepth, static.PenetrationDepth, exactGapTol) {
			t.Fatalf("v=%v: depth %.12g != static %.12g",
				v, res.PenetrationDepth, static.PenetrationDepth)
		}
		if !vecEq(res.Normal, static.Normal, absTol) {
			t.Fatalf("v=%v: normal %v != static %v", v, res.Normal, static.Normal)
		}
		if !vecEq(res.PointA, static.PointA, absTol) {
			t.Fatalf("v=%v: pointA %v != static %v", v, res.PointA, static.PointA)
		}
		if !vecEq(res.PointB, static.PointB, absTol) {
			t.Fatalf("v=%v: pointB %v != static %v", v, res.PointB, static.PointB)
		}
	}
}

// TestSweep_EndTouchAndNearMiss pins down the t=1 boundary that step sizing
// and the convergence test get wrong most easily:
//   - parts meeting exactly at t=1 report a contact time of 1;
//   - parts falling short by a hair report the window as safe, with the
//     closest instant at t=1 and the exact residual gap.
func TestSweep_EndTouchAndNearMiss(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(2, 0, 3, 1)

	// Exact touch at t=1: gap 1, closing speed 1.
	touch, err := Swept(SweptInput{A: a, B: b, VelocityB: Vec2{X: -1}})
	if err != nil {
		t.Fatal(err)
	}
	if touch.Status != SweptStatusContact {
		t.Fatalf("exact end touch: status = %s", touch.Status)
	}
	if !approxEq(touch.Time, 1.0, sweepTimeTolTc) {
		t.Fatalf("exact end touch: time = %.15g, want 1", touch.Time)
	}
	if !vecEq(touch.Normal, Vec2{X: 1}, sweepExactTol) {
		t.Fatalf("exact end touch: normal = %v", touch.Normal)
	}
	if !vecEq(touch.PointA, Vec2{X: 1, Y: 0}, sweepPointTol) &&
		touch.PointA.Y > 1+sweepPointTol {
		t.Fatalf("exact end touch: contact point %v off the facing edge", touch.PointA)
	}

	// Short by epsilon: closing speed 1-eps -> residual gap eps at t=1.
	for _, eps := range []float64{1e-3, 1e-6, 1e-7} {
		miss, err := Swept(SweptInput{A: a, B: b, VelocityB: Vec2{X: -(1 - eps)}})
		if err != nil {
			t.Fatal(err)
		}
		if miss.Status != SweptStatusSafe {
			t.Fatalf("near miss eps=%g: status = %s, want safe", eps, miss.Status)
		}
		if !approxEq(miss.Time, 1.0, sweepTimeTolTc) {
			t.Fatalf("near miss eps=%g: closest time = %.15g, want 1", eps, miss.Time)
		}
		if !approxEqRel(miss.Distance, eps, 1e-6, 1e-12) {
			t.Fatalf("near miss eps=%g: residual gap %.12g, want %g",
				eps, miss.Distance, eps)
		}
	}

	// The symmetric overshoot (closing speed 1+eps) must hit just before 1.
	for _, eps := range []float64{1e-3, 1e-6} {
		over, err := Swept(SweptInput{A: a, B: b, VelocityB: Vec2{X: -(1 + eps)}})
		if err != nil {
			t.Fatal(err)
		}
		wantT := 1.0 / (1 + eps)
		if over.Status != SweptStatusContact {
			t.Fatalf("overshoot eps=%g: status = %s", eps, over.Status)
		}
		if !approxEqRel(over.Time, wantT, 1e-6, 1e-12) {
			t.Fatalf("overshoot eps=%g: time %.12g, want %.12g", eps, over.Time, wantT)
		}
		if over.Time > 1 {
			t.Fatalf("overshoot impact time %.15g exceeds window", over.Time)
		}
	}
}

// TestSweep_Grazing_ContinuousCatchesSamplingMisses is the decisive test of
// the requirement: a uniform time grid evaluated with the static kernel never
// sees the grazing instant (the path is tangent to contact), while the
// continuous sweep reports the exact impact.
func TestSweep_Grazing_ContinuousCatchesSamplingMisses(t *testing.T) {
	// Fixture (re-derived here so the arithmetic is test-visible):
	// in A's frame, B's lower-left corner travels
	//   p(t) = (2.4,0.8) + t*(-3,3);
	// the A corner it grazes is (2,1).
	//   2.4 - 3t = 2 -> t = 2/15
	//   0.8 + 3t = 1 -> t = 1/15
	// Those differ, so instead choose the B start that hits *both* at the
	// same t*=1/3: p0 = (2,1) - t*w = (2+1, 1-1) = (3,0).
	tStar := 1.0 / 3.0
	w := Vec2{X: -3, Y: 3}
	p0 := Vec2{X: 2, Y: 1}.Sub(w.Scale(tStar)) // (3,0)
	if !vecEq(p0, Vec2{X: 3, Y: 0}, 1e-12) {
		t.Fatalf("fixture derivation: p0 = %v", p0)
	}
	// B is a 0.4-wide square with lower-left corner p0: [3,3.4]x[0,0.4].
	a := rect(0, 0, 2, 1)
	b := rect(3, 0, 3.4, 0.4)

	// Analytic cross-check of the geometry around t*: in A's frame the B
	// square overlaps A only while x-bands and y-bands overlap.
	relAt := func(tt float64) (bx0, by0 float64) {
		q := p0.Add(w.Scale(tt))
		return q.X, q.Y
	}
	// x band overlap: bx0 <= 2 and bx0+0.4 >= 0.
	// y band overlap: by0 <= 1 and by0+0.4 >= 0.
	overlapBands := func(tt float64) (bool, bool) {
		bx, by := relAt(tt)
		xOK := bx <= 2+1e-12 && bx+0.4 >= -1e-12
		yOK := by <= 1+1e-12 && by+0.4 >= -1e-12
		return xOK, yOK
	}
	// Contact interval in each axis separately:
	//   bx(t)=3-3t in [2-0.4,2]=[1.6,2] -> t in [1/3, 7/15]
	//   by(t)=0+3t in [1-0.4,1]=[0.6,1] -> t in [1/5, 1/3]
	xLo, xHi := 1.0/3.0, 7.0/15.0
	yLo, yHi := 1.0/5.0, 1.0/3.0
	if !(xLo < xHi && yLo < yHi && xLo == yHi) {
		t.Fatalf("fixture: bands do not meet at a single point")
	}
	for _, tt := range []float64{0, 0.1, 0.2, 0.3, tStar, 0.4, 0.5, 1} {
		xOK, yOK := overlapBands(tt)
		wantContact := math.Abs(tt-tStar) <= 1e-12
		if got := xOK && yOK; got != wantContact {
			t.Fatalf("analytic fixture wrong at t=%.3f: overlap=%v want %v",
				tt, got, wantContact)
		}
	}

	common := Vec2{X: 1.5, Y: 0.8}
	in := SweptInput{
		A:         a,
		B:         b,
		VelocityA: common,
		VelocityB: common.Add(w),
	}

	// 1) Uniform-grid static sampling at every dyadic partition up to 512
	//    must report a fully safe window: every sampled frame is strictly
	//    separated by a comfortable margin, and no frame is classified as a
	//    contact.
	for n := 2; n <= 512; n *= 2 {
		minGap := math.Inf(1)
		for k := 0; k <= n; k++ {
			tt := float64(k) / float64(n)
			r, err := Evaluate(posedAt(a, Vec2{}, common, tt),
				posedAt(b, Vec2{}, common.Add(w), tt))
			if err != nil {
				t.Fatalf("n=%d k=%d: %v", n, k, err)
			}
			if r.Status == StatusPenetrated {
				t.Fatalf("discretization n=%d unexpectedly hit a contact frame", n)
			}
			if r.Distance < minGap {
				minGap = r.Distance
			}
		}
		// The closest sampled gap is orders of magnitude above the kernel's
		// 1e-9 relative contact band: a grid genuinely cannot see this.
		if minGap < 100*sweepContactGapRel {
			t.Fatalf("n=%d grid came within %.3e of contact: grazing interval "+
				"not actually missed by sampling", n, minGap)
		}
	}

	// 2) The continuous solver catches the tangent contact...
	res, err := Swept(in)
	if err != nil {
		t.Fatalf("continuous sweep: %v", err)
	}
	if res.Status != SweptStatusContact {
		t.Fatalf("continuous sweep status = %s, want contact (grazing)", res.Status)
	}
	// The cusp gap is 3*sqrt(2)*|t-t*| (relative speed magnitude 3*sqrt2),
	// so the time accuracy is the kernel's geometric band (~1e-10 relative)
	// divided by that speed: assert a 1e-9 time tolerance, far tighter than
	// any uniform grid could localize (and exact for "does it touch").
	if !approxEq(res.Time, tStar, 1e-9) {
		t.Fatalf("grazing impact time = %.15g, want %.15g", res.Time, tStar)
	}
	// At a vertex–vertex tangent cusp the contact normal is directionally
	// singular: approaching from either side it tends to an edge normal
	// ((1,0) / (0,1)), and only exactly at the corner is every normal in the
	// 90-degree cone admissible. What is well defined — and what the response
	// must satisfy — is: a unit normal bisecting the two one-sided limits (a
	// robust solver returns the averaged diagonal (1,-1)/sqrt2 from its last
	// well-resolved separated frame), OR one of the axis limits when the
	// frame is within the kernel floor. Accept either, requiring it to be
	// unit and within the admissible cone between the two limits.
	if math.Abs(res.Normal.Len()-1) > 1e-9 {
		t.Fatalf("grazing normal not unit: %v (len %v)", res.Normal, res.Normal.Len())
	}
	if res.Normal.X < -1e-6 || res.Normal.Y > 1e-6 {
		t.Fatalf("grazing normal %v points outside the admissible corner cone",
			res.Normal)
	}
	// The contact point is the single shared corner, in world coordinates
	// (A corner (2,1) carried by the common velocity to t*). That location is
	// continuous across the cusp and is localized tightly.
	wantPoint := Vec2{X: 2, Y: 1}.Add(common.Scale(tStar))
	if !vecEq(res.PointA, wantPoint, 1e-7) {
		t.Fatalf("grazing contact point = %v, want %v", res.PointA, wantPoint)
	}
	if !vecEq(res.PointA, res.PointB, sweepPointTol) {
		t.Fatalf("grazing witnesses differ: %v vs %v", res.PointA, res.PointB)
	}
}

// TestSweep_InteriorClosestApproach: parts approach, miss, then recede within
// the window. The result must be safe, with an interior closest time and a
// minimum gap matching the analytic value.
func TestSweep_InteriorClosestApproach(t *testing.T) {
	// A unit square; B starts 2 units to the right and 0.5 above, moving
	// left at speed 2: in A's frame B = [2-2t,3-2t] x [1.5,2.5].
	// Vertical gap to A is constant 0.5; horizontal gap is |...|: closest at
	// t=0.75 (B left edge x=0.5? compute): overlap bands y: [1.5,2.5] vs
	// [0,1] never overlap -> corner distance.
	// B lower-left (2-2t, 1.5) approaches A corner (1,1) until 2-2t=1 ->
	// t=0.5; after that the horizontal distance grows while dy=0.5 fixed, so
	// the closest point to the (then horizontally overlapping) A edge x=1 is
	// vertical: gap 0.5 for t in [0.5,1.0]. Closest gap is therefore 0.5
	// across a plateau starting at t=0.5; first such instant t=0.5.
	in := SweptInput{
		A:         rect(0, 0, 1, 1),
		B:         rect(2, 1.5, 3, 2.5),
		VelocityB: Vec2{X: -2},
	}
	res, err := Swept(in)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != SweptStatusSafe {
		t.Fatalf("status = %s, want safe", res.Status)
	}
	if !approxEq(res.Distance, 0.5, sweepExactTol) {
		t.Fatalf("min gap = %.12g, want 0.5", res.Distance)
	}
	if res.Time < 0.5-sweepTimeTolTc || res.Time > 1+sweepTimeTolTc {
		t.Fatalf("closest time = %.12g, want t in [0.5,1]", res.Time)
	}
	// Across the plateau the minimum 0.5 gap is realized between A's top
	// edge (y=1) and B's bottom edge (y=1.5); any x within the horizontal
	// overlap is a valid witness, so assert the edge membership rather than
	// a single point.
	if !approxEq(res.PointA.Y, 1.0, sweepPointTol) {
		t.Fatalf("closest point on A = %v, want it on the top edge y=1", res.PointA)
	}
	if res.PointA.X < -sweepPointTol || res.PointA.X > 1+sweepPointTol {
		t.Fatalf("closest point on A = %v has x outside the top edge", res.PointA)
	}
	if !approxEq(res.PointB.Y-res.PointA.Y, 0.5, sweepPointTol) {
		t.Fatalf("witness vertical separation = %.12g, want 0.5",
			res.PointB.Y-res.PointA.Y)
	}

	// A genuinely single-instant interior minimum: same setup but B also
	// drifts up, so the vertical gap has a minimum at one time.
	// B lower-left = (2-2t, 1.5+0.5t); vertical gap = 0.5+0.5t (grows),
	// horizontal reaches zero at t=0.5. Closest corner distance
	// d(t)^2 = max(0,1-(2-2t))^2 + (0.5+0.5t)^2 for t<=0.5; minimize:
	// f(t)=(1-2t)^2+(0.5+0.5t)^2 ; f'= -4(1-2t)+(0.5+0.5t)=0
	// -> -4+8t+0.5+0.5t = 0 -> 8.5t = 3.5 -> t=7/17.
	in2 := SweptInput{
		A:         rect(0, 0, 1, 1),
		B:         rect(2, 1.5, 3, 2.5),
		VelocityB: Vec2{X: -2, Y: 0.5},
	}
	r2, err := Swept(in2)
	if err != nil {
		t.Fatal(err)
	}
	wantT := 7.0 / 17.0
	if r2.Status != SweptStatusSafe {
		t.Fatalf("status = %s, want safe", r2.Status)
	}
	if !approxEq(r2.Time, wantT, 1e-6) {
		t.Fatalf("closest time = %.12g, want %.12g", r2.Time, wantT)
	}
	dx := 1 - 2*wantT
	dy := 0.5 + 0.5*wantT
	wantD := math.Hypot(dx, dy)
	if !approxEqRel(r2.Distance, wantD, 1e-6, 1e-12) {
		t.Fatalf("min gap = %.12g, want %.12g", r2.Distance, wantD)
	}
}

// TestSweep_OffsetsHonored: initial pose offsets enter the posed geometry.
func TestSweep_OffsetsHonored(t *testing.T) {
	// B is modeled as a unit square at the origin and placed via an offset so
	// its lower-left corner starts at (2,2); it translates (-1,-1). A is the
	// unit square [0,1]^2. The moving corner (2-t,2-t) reaches A's top-right
	// corner (1,1) exactly at t=1: a clean corner contact at the window end.
	a := rect(0, 0, 1, 1)
	bLocal := rect(0, 0, 1, 1)
	res, err := Swept(SweptInput{
		A:         a,
		B:         bLocal,
		OffsetB:   Vec2{X: 2, Y: 2},
		VelocityB: Vec2{X: -1, Y: -1},
	})
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != SweptStatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if !approxEq(res.Time, 1.0, sweepTimeTolTc) {
		t.Fatalf("time = %.12g, want 1", res.Time)
	}
	// Contact normal is the corner diagonal, contact point the A corner.
	wantN := Vec2{X: 1, Y: 1}.Normalized()
	if !vecEq(res.Normal, wantN, 1e-7) && !vecEq(res.Normal, wantN.Scale(-1), 1e-7) {
		t.Fatalf("normal = %v, want %v", res.Normal, wantN)
	}
	if !vecEq(res.PointA, Vec2{X: 1, Y: 1}, 1e-6) {
		t.Fatalf("contact point = %v, want (1,1)", res.PointA)
	}

	// The identical relative motion expressed with the offset moved onto A
	// (with the compensating velocities) must give the same impact time.
	r2, err := Swept(SweptInput{
		A:         translate(a, Vec2{X: -2, Y: -2}),
		B:         bLocal,
		VelocityA: Vec2{X: 1, Y: 1},
		VelocityB: Vec2{},
	})
	if err != nil {
		t.Fatal(err)
	}
	if !approxEq(r2.Time, res.Time, sweepExactTol) || r2.Status != res.Status {
		t.Fatalf("offset equivalence broken: %+v vs %+v", r2, res)
	}

	// An initial gap produced purely by an offset is measured by the t=0
	// static frame: no motion, safe result carries that exact distance.
	r3, err := Swept(SweptInput{
		A: a, B: bLocal, OffsetB: Vec2{X: 3, Y: 0},
	})
	if err != nil {
		t.Fatal(err)
	}
	if r3.Status != SweptStatusSafe || !approxEq(r3.Distance, 2.0, exactGapTol) {
		t.Fatalf("offset-only gap: %+v", r3)
	}
}

// TestSweep_ValidationRejectsBadInput: every static-kernel validity rule
// carries over, and non-finite velocities are refused with a dedicated code.
func TestSweep_ValidationRejectsBadInput(t *testing.T) {
	good := rect(0, 0, 1, 1)
	cases := []struct {
		name string
		in   SweptInput
		code string
	}{
		{"A too few vertices", SweptInput{A: []Vec2{{0, 0}, {1, 1}}, B: good}, ErrTooFewVertices},
		{"B degenerate", SweptInput{A: good, B: []Vec2{{0, 0}, {1, 1}, {2, 2}}}, ErrDegeneratePolygon},
		{"A non convex", SweptInput{
			A: []Vec2{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}, B: good}, ErrNonConvexPolygon},
		{"A non finite vertex", SweptInput{
			A: []Vec2{{0, 0}, {1, 0}, {1, math.NaN()}}, B: good}, ErrNonFiniteCoordinate},
		{"velocity A NaN", SweptInput{A: good, B: good, VelocityA: Vec2{X: math.NaN()}}, ErrNonFiniteVelocity},
		{"velocity B +Inf", SweptInput{A: good, B: good, VelocityB: Vec2{Y: math.Inf(1)}}, ErrNonFiniteVelocity},
		{"velocity A -Inf", SweptInput{A: good, B: good, VelocityA: Vec2{X: math.Inf(-1)}}, ErrNonFiniteVelocity},
		{"offset A non finite", SweptInput{A: good, B: good, OffsetA: Vec2{Y: math.Inf(1)}}, ErrNonFiniteCoordinate},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := Swept(tc.in)
			assertCode(t, err, tc.code)
		})
	}
}

// TestSweep_BudgetExhaustedReportsError forces the iteration cap to zero and
// checks the sweep errors with SWEEP_NO_CONVERGENCE instead of looping or
// fabricating a "safe" verdict.
func TestSweep_BudgetExhaustedReportsError(t *testing.T) {
	orig := sweepMaxSteps
	sweepMaxSteps = 0
	defer func() { sweepMaxSteps = orig }()
	_, err := Swept(sweptHeadOn(2.0))
	assertCode(t, err, ErrSweepNoConvergence)
}

// TestSweep_IterationBudgetRespected runs many approaching configurations and
// asserts every answer stays within the declared evaluation budget.
func TestSweep_IterationBudgetRespected(t *testing.T) {
	a := rect(0, 0, 1, 1)
	for i := 0; i < 200; i++ {
		gap := 0.05 + float64(i%40)*0.1
		b := rect(1+gap, 0, 2+gap, 1)
		u := 0.3 + float64(i%17)*0.25
		res, err := Swept(SweptInput{A: a, B: b, VelocityB: Vec2{X: -u}})
		if err != nil {
			t.Fatalf("i=%d: %v", i, err)
		}
		if res.Iterations > MaxSweepIterations {
			t.Fatalf("i=%d exceeded budget: %d", i, res.Iterations)
		}
	}
}

// TestSweep_RandomConvexAgainstFineGrid cross-checks the continuous verdict
// on random convex polygons against a dense reference grid plus an analytic
// root refinement between any two sign-changing grid cells.
func TestSweep_RandomConvexAgainstFineGrid(t *testing.T) {
	rng := rand.New(rand.NewSource(20260928))
	for iter := 0; iter < 120; iter++ {
		a := randomConvex(rng, 3+rng.Intn(4), 2.0)
		b0 := randomConvex(rng, 3+rng.Intn(4), 1.0)
		// Place B away from A and give it a random relative velocity; add a
		// harmless common velocity to exercise that branch too.
		th := rng.Float64() * 2 * math.Pi
		off := Vec2{X: math.Cos(th), Y: math.Sin(th)}.Scale(4 + rng.Float64()*3)
		phi := rng.Float64() * 2 * math.Pi
		speed := 1.0 + rng.Float64()*7.0
		w := Vec2{X: math.Cos(phi), Y: math.Sin(phi)}.Scale(speed)
		common := Vec2{X: rng.NormFloat64(), Y: rng.NormFloat64()}.Scale(2.0)

		in := SweptInput{
			A:         a,
			B:         b0,
			OffsetB:   off,
			VelocityA: common,
			VelocityB: common.Add(w),
		}
		res, err := Swept(in)
		if err != nil {
			t.Fatalf("iter %d: %v", iter, err)
		}

		// Reference: scan a fine grid in A's frame; on a separated->contact
		// sign change refine the first cell by dense subdivision.
		const N = 4000
		firstHit := math.Inf(1)
		gridMinGap := math.Inf(1)
		var gridTMin float64
		prevSep := true
		var prevT float64
		for k := 0; k <= N; k++ {
			tt := float64(k) / float64(N)
			r := mustEval(t, posedAt(a, Vec2{}, Vec2{}, tt),
				posedAt(b0, off, w, tt))
			if r.Status == StatusSeparated {
				if r.Distance < gridMinGap {
					gridMinGap, gridTMin = r.Distance, tt
				}
			}
			if r.Status == StatusPenetrated {
				if prevSep {
					// Densify this cell to locate the entry.
					for j := 1; j < 200; j++ {
						u := prevT + (tt-prevT)*float64(j)/200.0
						rr := mustEval(t, posedAt(a, Vec2{}, Vec2{}, u),
							posedAt(b0, off, w, u))
						if rr.Status == StatusPenetrated {
							firstHit = u
							break
						}
					}
					if math.IsInf(firstHit, 1) {
						firstHit = tt
					}
				}
				prevSep = false
			} else {
				prevSep = true
			}
			prevT = tt
		}

		switch {
		case !math.IsInf(firstHit, 1):
			if res.Status != SweptStatusContact {
				t.Fatalf("iter %d: grid found contact at t~%.6f, sweep says safe",
					iter, firstHit)
			}
			if math.Abs(res.Time-firstHit) > 1e-3 {
				t.Fatalf("iter %d: impact %.9g disagrees with grid %.9g",
					iter, res.Time, firstHit)
			}
			if res.Time < -1e-12 || res.Time > 1+1e-12 {
				t.Fatalf("iter %d: impact time out of window: %v", iter, res.Time)
			}
		default:
			if res.Status != SweptStatusSafe {
				t.Fatalf("iter %d: sweep reports contact at %.6f, dense grid is safe (min gap %.3e at %.4f)",
					iter, res.Time, gridMinGap, gridTMin)
			}
			// Reported minimum gap should match the grid minimum closely.
			if math.Abs(res.Distance-gridMinGap) > 1e-5*math.Max(1, gridMinGap) {
				t.Fatalf("iter %d: min gap %.9g != grid %.9g (t %.4f vs %.4f)",
					iter, res.Distance, gridMinGap, res.Time, gridTMin)
			}
		}
	}
}

func mustEval(t *testing.T, a, b []Vec2) *Result {
	t.Helper()
	r, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("reference evaluate: %v", err)
	}
	return r
}
