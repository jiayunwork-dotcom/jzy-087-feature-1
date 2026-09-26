package geometry

import (
	"math"
	"testing"
)

// Tolerances are deliberately stated up front rather than asserting "some
// value is returned":
//   - exactGapTol: 1e-12, for analytically known edge-to-edge gaps/depths;
//   - relTol/absTol: 1e-9, for transformed (translated) geometries whose
//     witness points are still expected to agree tightly.
const (
	exactGapTol = 1e-12
	absTol      = 1e-9
	relTol      = 1e-9
)

func approxEq(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

func approxEqRel(a, b, rel, abs float64) bool {
	return math.Abs(a-b) <= abs+rel*math.Max(math.Abs(a), math.Abs(b))
}

func vecEq(a, b Vec2, tol float64) bool {
	return approxEq(a.X, b.X, tol) && approxEq(a.Y, b.Y, tol)
}

func unitV(v Vec2) Vec2 { return v.Normalized() }

func rect(x0, y0, x1, y1 float64) []Vec2 {
	return []Vec2{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
}

func tri(p0, p1, p2 Vec2) []Vec2 { return []Vec2{p0, p1, p2} }

func translate(pts []Vec2, t Vec2) []Vec2 {
	out := make([]Vec2, len(pts))
	for i, p := range pts {
		out[i] = p.Add(t)
	}
	return out
}

// TestValidation_TooFewVertices covers the < 3 vertices rule.
func TestValidation_TooFewVertices(t *testing.T) {
	cases := []struct {
		name string
		pts  []Vec2
	}{
		{"empty", nil},
		{"one", []Vec2{{0, 0}}},
		{"two", []Vec2{{0, 0}, {1, 1}}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := NewPolygon(tc.pts)
			assertCode(t, err, ErrTooFewVertices)
		})
	}
}

// TestValidation_Degenerate covers zero-area / duplicated-vertex inputs.
func TestValidation_Degenerate(t *testing.T) {
	cases := []struct {
		name string
		pts  []Vec2
	}{
		{
			"collinear",
			[]Vec2{{0, 0}, {1, 0}, {2, 0}, {3, 0}},
		},
		{
			"collinear points as triangle",
			[]Vec2{{0, 0}, {1, 1}, {2, 2}},
		},
		{
			"consecutive duplicate",
			[]Vec2{{0, 0}, {0, 0}, {1, 0}, {1, 1}},
		},
		{
			"repeated first vertex at end",
			[]Vec2{{0, 0}, {1, 0}, {1, 1}, {0, 0}},
		},
		{
			"zero-area sliver",
			[]Vec2{{0, 0}, {1, 0}, {2, 1e-13}, {3, 0}},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := NewPolygon(tc.pts)
			assertCode(t, err, ErrDegeneratePolygon)
		})
	}
}

// TestValidation_NonConvex ensures concave contours are refused outright and
// no convex decomposition is attempted.
func TestValidation_NonConvex(t *testing.T) {
	// Arrow-head concave pentagon (an L-shaped notch):
	// (0,0)->(3,0)->(3,1)->(1,1)->(1,3)->(0,3) closes back; the turn at
	// (1,1) is reflex.
	concave := []Vec2{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}
	_, err := NewPolygon(concave)
	assertCode(t, err, ErrNonConvexPolygon)

	// A classic bowtie (self-intersecting quadrilateral) must be rejected;
	// depending on how its cancellation manifests it reads as non-convex or
	// zero-area degenerate, but never as an accepted polygon.
	bowtie := []Vec2{{0, 0}, {2, 2}, {2, 0}, {0, 2}}
	_, err = NewPolygon(bowtie)
	if err == nil {
		t.Fatalf("bowtie contour accepted as a polygon")
	}
	if ke, ok := err.(*KernelError); ok {
		if ke.Code != ErrNonConvexPolygon && ke.Code != ErrDegeneratePolygon {
			t.Fatalf("bowtie: code = %s, want NON_CONVEX_POLYGON or DEGENERATE_POLYGON", ke.Code)
		}
	} else {
		t.Fatalf("bowtie: unexpected error type %T", err)
	}

	// Reversed winding of the concave polygon must also fail.
	rev := make([]Vec2, len(concave))
	for i := range concave {
		rev[i] = concave[len(concave)-1-i]
	}
	_, err = NewPolygon(rev)
	assertCode(t, err, ErrNonConvexPolygon)
}

// TestValidation_NonFinite rejects NaN/Inf payloads with a readable code.
func TestValidation_NonFinite(t *testing.T) {
	_, err := NewPolygon([]Vec2{{0, 0}, {1, 0}, {1, math.NaN()}})
	assertCode(t, err, ErrNonFiniteCoordinate)

	_, err = NewPolygon([]Vec2{{0, 0}, {math.Inf(1), 0}, {1, 1}})
	assertCode(t, err, ErrNonFiniteCoordinate)
}

// TestValidation_AcceptsConvexWindingAndCollinear ensures legitimate convex
// inputs (either winding, with boundary collinear vertices) are accepted.
func TestValidation_AcceptsConvexWindingAndCollinear(t *testing.T) {
	squareCW := []Vec2{{0, 0}, {0, 1}, {1, 1}, {1, 0}}
	p, err := NewPolygon(squareCW)
	if err != nil {
		t.Fatalf("CW square rejected: %v", err)
	}
	if p.CCW {
		t.Fatalf("expected CW orientation flag = false")
	}

	// Convex rectangle with an extra collinear vertex on one edge.
	withCollinear := []Vec2{{0, 0}, {0.5, 0}, {1, 0}, {1, 1}, {0, 1}}
	if _, err := NewPolygon(withCollinear); err != nil {
		t.Fatalf("convex polygon with collinear boundary point rejected: %v", err)
	}
}

func assertCode(t *testing.T, err error, code string) {
	t.Helper()
	if err == nil {
		t.Fatalf("expected error %s, got nil", code)
	}
	ke, ok := err.(*KernelError)
	if !ok {
		t.Fatalf("expected *KernelError, got %T: %v", err, err)
	}
	if ke.Code != code {
		t.Fatalf("expected code %s, got %s (%s)", code, ke.Code, ke.Message)
	}
	if ke.Message == "" {
		t.Fatalf("error code %s carries no readable message", code)
	}
}
