package geometry

import (
	"math"
	"sort"
)

// MaxEPAIterations bounds the polytope expansion steps. EPA on two polygons
// expands by hull vertices and terminates quickly; an exhausted budget is an
// error, never a silent zero-distance "separated" answer.
const MaxEPAIterations = 256

// epaMaxSteps is the live bound, indirected so tests can force exhaustion.
var epaMaxSteps = MaxEPAIterations

// epaContact is the result of expanding a simplex that contains the origin:
// penetration depth and the separating direction, with contact points.
type epaContact struct {
	// depth is the distance from the origin to the closest CSO boundary.
	depth float64
	// normal points out of A⊖B at the closest edge, i.e. the direction in
	// which B must move to separate the parts (the contact normal from A
	// toward B).
	normal Vec2
	// pointA / pointB are the contact (witness) points on the two polygons.
	pointA, pointB Vec2
	steps          int
}

// epa performs the Expanding Polytope Algorithm on a simplex that contains
// the origin.
func epa(a, b *Polygon, seed []SupportPoint, gjkSteps int) (*epaContact, error) {
	tol := epsFor(a.maxLen(), b.maxLen())
	verts := bootstrapPolytope(a, b, seed, tol)
	if verts == nil {
		return nil, &KernelError{Code: ErrEPAFailure,
			Message: "EPA could not build a polytope enclosing the origin"}
	}

	stopGap := convergenceRel * polytopeScale(verts)
	if stopGap < tol {
		stopGap = tol
	}

	for step := 1; step <= epaMaxSteps; step++ {
		edge, normalOut, dist := closestEdge(verts)
		if edge < 0 {
			return nil, &KernelError{Code: ErrEPAFailure,
				Message: "EPA found no usable polytope edge"}
		}

		sp := support(a, b, normalOut)
		advance := sp.V.Dot(normalOut) - dist
		if advance <= stopGap || duplicatesAny(sp, verts, tol) {
			ca, cb := edgeWitnesses(verts[edge], verts[(edge+1)%len(verts)], tol)
			return &epaContact{
				depth:  max(dist, 0),
				normal: normalOut,
				pointA: ca,
				pointB: cb,
				steps:  gjkSteps + step,
			}, nil
		}
		// Expand the polytope: rebuild the convex hull with the new
		// support vertex. In 2D the hull carries only a handful of the
		// polygons' vertices, so a hull rebuild is cheap and guarantees a
		// valid CCW boundary.
		verts = convexHull(append(verts, sp), tol)
	}
	return nil, &KernelError{Code: ErrEPANoConvergence,
		Message: "EPA polytope expansion did not converge within the step budget"}
}

// bootstrapPolytope turns a 1-, 2- or 3-point origin-containing simplex into
// a CCW-ordered polytope whose interior (or boundary) contains the origin.
// All vertices are support points, hence lie on the boundary of the CSO.
func bootstrapPolytope(a, b *Polygon, seed []SupportPoint, tol float64) []SupportPoint {
	verts := dedupe(seed, tol)

	switch len(verts) {
	case 1:
		// A one-point seed means that support point is the origin. Pull
		// in four axis support points to form a genuine polytope.
		for _, d := range []Vec2{{X: 1}, {Y: 1}, {X: -1}, {Y: -1}} {
			verts = appendUnique(verts, support(a, b, d), tol)
		}
	case 2:
		// Expand to both sides of the segment (the origin lies on it).
		n := verts[1].V.Sub(verts[0].V).PerpLeft().Normalized()
		verts = appendUnique(verts, support(a, b, n), tol)
		verts = appendUnique(verts, support(a, b, n.Scale(-1)), tol)
	}

	// Vertices are CSO boundary points; reduce them to their convex hull
	// (CCW). A polar-angle order is NOT a convex order when the origin is
	// interior, hence a monotone-chain hull.
	ordered := convexHull(verts, tol)
	if len(ordered) >= 3 && polygonContainsOrigin(ordered, tol) {
		return ordered
	}

	// Defensive fallback for awkward degeneracies: sample more directions.
	for _, d := range []Vec2{
		{X: 1, Y: 1}, {X: 1, Y: -1}, {X: -1, Y: 1}, {X: -1, Y: -1},
		{X: 1, Y: 2}, {X: 2, Y: -1}, {X: -2, Y: 1}, {X: -1, Y: -2},
	} {
		sp := support(a, b, d)
		if !duplicatesAny(sp, ordered, tol) {
			ordered = convexHull(append(ordered, sp), tol)
			if polygonContainsOrigin(ordered, tol) {
				return ordered
			}
		}
	}
	if polygonContainsOrigin(ordered, tol) {
		return ordered
	}
	return nil
}

// convexHull returns the points in CCW convex-hull order using Andrew's
// monotone chain. Collinear boundary points are kept off the hull; this is
// exactly what EPA wants (edges are hull edges).
func convexHull(pts []SupportPoint, tol float64) []SupportPoint {
	if len(pts) <= 1 {
		return append([]SupportPoint(nil), pts...)
	}
	sorted := append([]SupportPoint(nil), pts...)
	sort.Slice(sorted, func(i, j int) bool {
		if sorted[i].V.X != sorted[j].V.X {
			return sorted[i].V.X < sorted[j].V.X
		}
		return sorted[i].V.Y < sorted[j].V.Y
	})
	cross := func(o, a, b SupportPoint) float64 {
		return a.V.Sub(o.V).Cross(b.V.Sub(o.V))
	}
	lower := make([]SupportPoint, 0, len(sorted))
	for _, p := range sorted {
		for len(lower) >= 2 && cross(lower[len(lower)-2], lower[len(lower)-1], p) <= tol*tol {
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, p)
	}
	upper := make([]SupportPoint, 0, len(sorted))
	for i := len(sorted) - 1; i >= 0; i-- {
		p := sorted[i]
		for len(upper) >= 2 && cross(upper[len(upper)-2], upper[len(upper)-1], p) <= tol*tol {
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, p)
	}
	// First/last points are duplicated between the two chains.
	return append(lower[:len(lower)-1], upper[:len(upper)-1]...)
}

func dedupe(verts []SupportPoint, tol float64) []SupportPoint {
	out := make([]SupportPoint, 0, len(verts))
	for _, v := range verts {
		if !duplicatesAny(v, out, tol) {
			out = append(out, v)
		}
	}
	return out
}

func appendUnique(verts []SupportPoint, sp SupportPoint, tol float64) []SupportPoint {
	if !duplicatesAny(sp, verts, tol) {
		return append(verts, sp)
	}
	return verts
}

// closestEdge returns the index of the CCW polytope edge whose supporting
// line is nearest to the origin, together with its outward unit normal and
// the (non-negative) distance of the origin to the line.
func closestEdge(verts []SupportPoint) (int, Vec2, float64) {
	bestI := -1
	bestD := math.Inf(1)
	var bestN Vec2
	for i := 0; i < len(verts); i++ {
		p := verts[i].V
		q := verts[(i+1)%len(verts)].V
		e := q.Sub(p)
		if e.Len2() == 0 {
			continue
		}
		// CCW edge: the left normal points inward, so the outward normal
		// is -PerpLeft.
		n := e.PerpLeft().Normalized().Scale(-1)
		d := n.Dot(p)
		if d < bestD {
			bestD = d
			bestI = i
			bestN = n
		}
	}
	if bestD < 0 {
		bestD = 0 // numerical guard: origin must be inside the polytope
	}
	return bestI, bestN, bestD
}

// edgeWitnesses recovers contact points on A and B by interpolating the
// witness vertices of the closest CSO edge at the edge point nearest the
// origin.
func edgeWitnesses(s1, s2 SupportPoint, tol float64) (Vec2, Vec2) {
	e := s2.V.Sub(s1.V)
	t := 0.0
	if e.Len2() > tol*tol {
		t = clampUnit(s1.V.Scale(-1).Dot(e) / e.Len2())
	}
	pa := s1.Pa.Add(s2.Pa.Sub(s1.Pa).Scale(t))
	pb := s1.Pb.Add(s2.Pb.Sub(s1.Pb).Scale(t))
	return pa, pb
}

func polytopeScale(verts []SupportPoint) float64 {
	m := 0.0
	for _, v := range verts {
		if l := v.V.Len(); l > m {
			m = l
		}
	}
	if m < 1 {
		return 1
	}
	return m
}

func polygonContainsOrigin(verts []SupportPoint, tol float64) bool {
	if len(verts) < 3 || signedArea(verts) <= 0 {
		return false
	}
	return originInsideCCW(verts, tol)
}

// originInsideCCW reports whether the origin is on/inside a CCW polygon.
func originInsideCCW(verts []SupportPoint, tol float64) bool {
	n := len(verts)
	for i := 0; i < n; i++ {
		e := verts[(i+1)%n].V.Sub(verts[i].V)
		// inward normal is PerpLeft; signed outside amount:
		if e.PerpLeft().Dot(verts[i].V.Scale(-1)) < -tol*max(1.0, e.Len()) {
			return false
		}
	}
	return true
}

func signedArea(verts []SupportPoint) float64 {
	a := 0.0
	for i := 0; i < len(verts); i++ {
		a += verts[i].V.Cross(verts[(i+1)%len(verts)].V)
	}
	return a
}
