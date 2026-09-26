package geometry

import (
	"math"
	"testing"
)

// TestKnownGap_AxisAlignedRectangles is the hand-checkable example shipped
// with the service: two axis-aligned rectangles with a known gap.
//
//	A = [0,2]x[0,1], B = [2.75,4.25]x[-0.5,1.5].
//
// The facing edges are x=2 (A) and x=2.75 (B); the exact clearance is 0.75
// and the contact normal is +X.
func TestKnownGap_AxisAlignedRectangles(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(2.75, -0.5, 4.25, 1.5)
	res, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("evaluate: %v", err)
	}
	if res.Status != StatusSeparated {
		t.Fatalf("status = %s, want separated", res.Status)
	}
	if !approxEq(res.Distance, 0.75, exactGapTol) {
		t.Fatalf("distance = %.15g, want 0.75", res.Distance)
	}
	if !vecEq(res.Normal, Vec2{X: 1}, exactGapTol) {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	// Closest pair must lie on the facing edges and be exactly the gap apart.
	if res.PointA.X != 2 {
		t.Fatalf("closest point on A = %v, expected x=2 edge", res.PointA)
	}
	if !approxEq(res.PointB.X, 2.75, exactGapTol) {
		t.Fatalf("closest point on B = %v, expected x=2.75 edge", res.PointB)
	}
	if !approxEq(res.PointA.Y, res.PointB.Y, exactGapTol) {
		t.Fatalf("witness points not aligned: %v vs %v", res.PointA, res.PointB)
	}
	if d := res.PointA.Sub(res.PointB).Len(); !approxEq(d, res.Distance, exactGapTol) {
		t.Fatalf("witness pair distance %.15g != reported distance %.15g", d, res.Distance)
	}
}

// TestAxisAlignedGaps_Table checks several analytically known edge-to-edge
// gaps, including both axis directions and a corner-to-corner gap.
func TestAxisAlignedGaps_Table(t *testing.T) {
	cases := []struct {
		name       string
		a, b       []Vec2
		wantDist   float64
		wantNormal Vec2
	}{
		{"gap x 0.5", rect(0, 0, 1, 1), rect(1.5, 0, 2.5, 1), 0.5, Vec2{1, 0}},
		{"gap x 2", rect(0, 0, 1, 1), rect(3, -1, 4, 2), 2, Vec2{1, 0}},
		{"gap y 1.25", rect(0, 0, 2, 1), rect(0.25, 2.25, 1.5, 3), 1.25, Vec2{0, 1}},
		{"gap y negative side", rect(0, -3, 1, -2), rect(0, 0, 1, 1), 2, Vec2{0, 1}},
		{"corner to corner sqrt2", rect(0, 0, 1, 1), rect(2, 2, 3, 3), math.Sqrt2, Vec2{1, 1}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := Evaluate(tc.a, tc.b)
			if err != nil {
				t.Fatalf("evaluate: %v", err)
			}
			if res.Status != StatusSeparated {
				t.Fatalf("status = %s, want separated", res.Status)
			}
			if !approxEq(res.Distance, tc.wantDist, exactGapTol) {
				t.Fatalf("distance = %.15g, want %.15g", res.Distance, tc.wantDist)
			}
			wantN := unitV(tc.wantNormal)
			if !vecEq(res.Normal, wantN, exactGapTol) {
				t.Fatalf("normal = %v, want %v", res.Normal, wantN)
			}
			// Witness pair must realize the reported distance exactly.
			if d := res.PointA.Sub(res.PointB).Len(); !approxEqRel(d, res.Distance, relTol, absTol) {
				t.Fatalf("witness distance %.12g disagrees with reported %.12g", d, res.Distance)
			}
			// Normal must point from the A witness toward the B witness.
			nb := res.PointB.Sub(res.PointA).Normalized()
			if !vecEq(res.Normal, nb, 1e-8) {
				t.Fatalf("normal %v does not point from pointA %v to pointB %v (got %v)",
					res.Normal, res.PointA, res.PointB, nb)
			}
		})
	}
}

// TestTranslationChangesGapByDisplacement is the required relation: moving
// one part away from the other along the separating normal by s increases
// the reported clearance by approximately s (and moving toward it decreases
// it by the same amount until contact).
func TestTranslationChangesGapByDisplacement(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(2.75, -0.5, 4.25, 1.5)
	base, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("base evaluate: %v", err)
	}
	if base.Status != StatusSeparated {
		t.Fatalf("base should be separated")
	}

	for _, s := range []float64{0.1, 0.37, 1.0, 2.5} {
		moved := translate(b, base.Normal.Scale(s))
		res, err := Evaluate(a, moved)
		if err != nil {
			t.Fatalf("evaluate after +%.2f: %v", s, err)
		}
		want := base.Distance + s
		if !approxEqRel(res.Distance, want, relTol, absTol) {
			t.Fatalf("moving B by +%.3f along normal: distance %.12g, want %.12g",
				s, res.Distance, want)
		}
		// Normal is invariant under a translation along itself.
		if !vecEq(res.Normal, base.Normal, absTol) {
			t.Fatalf("normal changed after translation: %v vs %v", res.Normal, base.Normal)
		}
	}

	// Moving toward A by less than the gap shrinks the clearance by s.
	for _, s := range []float64{0.1, 0.5, 0.7} {
		moved := translate(b, base.Normal.Scale(-s))
		res, err := Evaluate(a, moved)
		if err != nil {
			t.Fatalf("evaluate after -%.2f: %v", s, err)
		}
		want := base.Distance - s
		if !approxEqRel(res.Distance, want, relTol, absTol) {
			t.Fatalf("moving B by -%.3f along normal: distance %.12g, want %.12g",
				s, res.Distance, want)
		}
	}

	// Moving by exactly the gap closes it: the result must be reported as
	// penetration with depth ~0, never as "separated with distance 0".
	moved := translate(b, base.Normal.Scale(-base.Distance))
	res, err := Evaluate(a, moved)
	if err != nil {
		t.Fatalf("evaluate at contact: %v", err)
	}
	if res.Status != StatusPenetrated {
		t.Fatalf("touching parts reported as %s; must be penetration branch", res.Status)
	}
	if res.PenetrationDepth > 1e-7 {
		t.Fatalf("contact depth = %.3e, want ~0", res.PenetrationDepth)
	}
}

// TestCommonTranslationInvariance: applying the same rigid translation to
// both parts leaves the gap, normal and relative witness geometry unchanged.
func TestCommonTranslationInvariance(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(2.75, -0.5, 4.25, 1.5)
	base, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("base: %v", err)
	}
	shifts := []Vec2{
		{X: 5, Y: 7},
		{X: -13.25, Y: 4.5},
		{X: 1000, Y: -2000},
		{X: 0.333, Y: 1.777},
	}
	for _, sh := range shifts {
		res, err := Evaluate(translate(a, sh), translate(b, sh))
		if err != nil {
			t.Fatalf("shift %v: %v", sh, err)
		}
		if res.Status != base.Status {
			t.Fatalf("shift %v changed status: %s", sh, res.Status)
		}
		if !approxEqRel(res.Distance, base.Distance, relTol, absTol) {
			t.Fatalf("shift %v changed distance: %.12g vs %.12g", sh, res.Distance, base.Distance)
		}
		if !vecEq(res.Normal, base.Normal, absTol) {
			t.Fatalf("shift %v changed normal: %v vs %v", sh, res.Normal, base.Normal)
		}
		// Witness points translate by exactly the same vector.
		if !vecEq(res.PointA, base.PointA.Add(sh), absTol) {
			t.Fatalf("shift %v: pointA = %v, want %v", sh, res.PointA, base.PointA.Add(sh))
		}
		if !vecEq(res.PointB, base.PointB.Add(sh), absTol) {
			t.Fatalf("shift %v: pointB = %v, want %v", sh, res.PointB, base.PointB.Add(sh))
		}
	}
}

// TestPenetrationDepth_RectangleOverlaps: for overlapping rectangles the
// reported penetration depth must equal the smaller overlap dimension
// (minimum translational vector distance), and the normal must align with
// that axis. Both relative orientations are tested.
func TestPenetrationDepth_RectangleOverlaps(t *testing.T) {
	cases := []struct {
		name      string
		a, b      []Vec2
		wantDepth float64
		wantAxis  Vec2 // unoriented allowed: ± accepted
	}{
		{
			name:      "x overlap 0.3 smaller than y overlap 0.8",
			a:         rect(0, 0, 2, 1),
			b:         rect(1.7, 0.1, 3.7, 1.9), // x overlap .3, y overlap .9
			wantDepth: 0.3,
			wantAxis:  Vec2{1, 0},
		},
		{
			name:      "y overlap smaller",
			a:         rect(0, 0, 4, 2),
			b:         rect(0.5, 1.6, 3, 3.6), // x overlap 2.5, y overlap .4
			wantDepth: 0.4,
			wantAxis:  Vec2{0, 1},
		},
		{
			name:      "equal unit squares exactly coincident -> depth 1",
			a:         rect(0, 0, 1, 1),
			b:         rect(0, 0, 1, 1),
			wantDepth: 1,
			wantAxis:  Vec2{1, 0}, // any face normal; axis-aligned accepted
		},
		{
			name:      "diagonal shift, x overlap 0.6 smaller than y 0.9",
			a:         rect(-2, -2, 1, 2),
			b:         rect(0.4, -1.1, 3, 1.9),
			wantDepth: 0.6,
			wantAxis:  Vec2{1, 0},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res, err := Evaluate(tc.a, tc.b)
			if err != nil {
				t.Fatalf("evaluate: %v", err)
			}
			if res.Status != StatusPenetrated {
				t.Fatalf("status = %s, want penetrated", res.Status)
			}
			if !approxEq(res.PenetrationDepth, tc.wantDepth, exactGapTol) {
				t.Fatalf("depth = %.15g, want %.15g", res.PenetrationDepth, tc.wantDepth)
			}
			axis := unitV(tc.wantAxis)
			if tc.name == "equal unit squares exactly coincident -> depth 1" {
				// Any of the four face normals is a valid MTV for
				// coincident squares; require it to be axis-aligned.
				if math.Abs(math.Abs(res.Normal.X)-1) > 1e-8 &&
					math.Abs(math.Abs(res.Normal.Y)-1) > 1e-8 {
					t.Fatalf("normal = %v, want an axis-aligned face normal", res.Normal)
				}
			} else if !vecEq(res.Normal, axis, 1e-8) && !vecEq(res.Normal, axis.Scale(-1), 1e-8) {
				t.Fatalf("normal = %v, want ±%v", res.Normal, axis)
			}
			if math.Abs(res.Normal.Len()-1) > 1e-12 {
				t.Fatalf("normal not unit: %v len=%v", res.Normal, res.Normal.Len())
			}
			// Witness points must be contact points lying on both polygons.
			if !pointInOrOn(t, tc.a, res.PointA, 1e-9) {
				t.Fatalf("contact point A %v is not on/inside A", res.PointA)
			}
			if !pointInOrOn(t, tc.b, res.PointB, 1e-9) {
				t.Fatalf("contact point B %v is not on/inside B", res.PointB)
			}
		})
	}
}

// TestSeparatingTranslationExactlyRemovesPenetration verifies the operational
// meaning of the reported penetration branch: translating B along the
// reported normal by exactly the penetration depth yields a zero-gap touch,
// and a hair more separates the bodies with the gap growing from zero.
func TestSeparatingTranslationExactlyRemovesPenetration(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(1.7, 0.1, 3.7, 1.9)
	res, err := Evaluate(a, b)
	if err != nil {
		t.Fatal(err)
	}
	push := res.Normal.Scale(res.PenetrationDepth)
	touched := translate(b, push)
	r2, err := Evaluate(a, touched)
	if err != nil {
		t.Fatalf("at touch: %v", err)
	}
	if r2.Status != StatusPenetrated || r2.PenetrationDepth > 1e-7 {
		t.Fatalf("after MTV translation: status=%s depth=%.3e, want touch (depth≈0)",
			r2.Status, r2.PenetrationDepth)
	}

	freed := translate(b, res.Normal.Scale(res.PenetrationDepth+0.25))
	r3, err := Evaluate(a, freed)
	if err != nil {
		t.Fatalf("freed: %v", err)
	}
	if r3.Status != StatusSeparated || !approxEq(r3.Distance, 0.25, 1e-8) {
		t.Fatalf("after MTV+0.25: status=%s dist=%.6g, want separated gap 0.25",
			r3.Status, r3.Distance)
	}
}

// TestTrianglesSanity checks the kernel on non-rectangle convex parts.
func TestTrianglesSanity(t *testing.T) {
	a := tri(Vec2{0, 0}, Vec2{4, 0}, Vec2{0, 4})
	// B is a small triangle in the x+y>=4 half-plane touching A's
	// hypotenuse x+y=4 exactly at the vertex (4,4) -> (4,4) has x+y=8, so
	// instead place B so one vertex lies on the line: (2.5,1.5) is on it,
	// and the rest of B stays on the x+y>=4 side.
	b := tri(Vec2{2.5, 1.5}, Vec2{3.5, 1.5}, Vec2{2.5, 2.5})
	res, err := Evaluate(a, b)
	if err != nil {
		t.Fatal(err)
	}
	// Parts touch at a single point: penetration branch with depth ~0,
	// never a "separated with distance 0".
	if res.Status != StatusPenetrated {
		t.Fatalf("status = %s, want penetrated (touch)", res.Status)
	}
	if res.PenetrationDepth > 1e-8 {
		t.Fatalf("touch depth = %.3e, want 0", res.PenetrationDepth)
	}

	// Shift B along the hypotenuse outward normal (1,1)/sqrt2 by sqrt2 ->
	// gap exactly 1.
	shift := Vec2{1, 1}
	res2, err := Evaluate(a, translate(b, shift))
	if err != nil {
		t.Fatal(err)
	}
	if res2.Status != StatusSeparated || !approxEq(res2.Distance, math.Sqrt2, 1e-9) {
		t.Fatalf("shifted triangles: status=%s dist=%.6g, want separated sqrt2",
			res2.Status, res2.Distance)
	}
	// The witness on A must lie on the hypotenuse x+y=4.
	if !approxEq(res2.PointA.X+res2.PointA.Y, 4, 1e-8) {
		t.Fatalf("witness on A %v not on hypotenuse", res2.PointA)
	}
}

// pointInOrOn is a strict convex containment test used only for sanity
// checking witness points in tests.
func pointInOrOn(t *testing.T, pts []Vec2, p Vec2, tol float64) bool {
	t.Helper()
	pg, err := NewPolygon(pts)
	if err != nil {
		t.Fatalf("invalid fixture polygon: %v", err)
	}
	vs := pg.Vertices
	if !pg.CCW {
		reversed := make([]Vec2, len(vs))
		for i := range vs {
			reversed[i] = vs[len(vs)-1-i]
		}
		vs = reversed
	}
	for i := range vs {
		e := vs[(i+1)%len(vs)].Sub(vs[i])
		if c := e.Cross(p.Sub(vs[i])); c < -tol*math.Max(1, e.Len()) {
			return false // CCW contour: interior requires c >= 0
		}
	}
	return true
}
