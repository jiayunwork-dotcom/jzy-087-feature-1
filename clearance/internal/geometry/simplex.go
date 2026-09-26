package geometry

import "math"

// evolveLine performs one GJK simplex update for a two-vertex simplex
// (segment). The slice stores the newest support point first: s[0]=A, s[1]=B.
//
// It returns the reduced simplex, the next search direction (pointing from
// the closest feature toward the origin) and a flag that is true when the
// segment already contains the origin (boundary contact, zero distance).
func evolveLine(s []SupportPoint, tol float64) (contained bool, out []SupportPoint, dir Vec2) {
	a, b := s[0].V, s[1].V
	ab := b.Sub(a)
	if ab.Len2() <= tol*tol {
		// Degenerate segment: a single point.
		ao := a.Scale(-1)
		if ao.Len() <= tol {
			return true, s[:1], Vec2{}
		}
		return false, s[:1], ao
	}
	t := clampUnit(a.Scale(-1).Dot(ab) / ab.Len2())
	switch {
	case t <= paramTol:
		// Voronoi region of A.
		if a.Len() <= tol {
			return true, s[:1], Vec2{}
		}
		return false, s[:1], a.Scale(-1)
	case t >= 1-paramTol:
		// Voronoi region of B.
		if b.Len() <= tol {
			return true, s[1:], Vec2{}
		}
		return false, s[1:], b.Scale(-1)
	default:
		// Interior Voronoi region of the segment.
		q := a.Add(ab.Scale(t))
		d := q.Scale(-1)
		if d.Len() <= tol {
			// Origin lies on the segment: the CSO boundary touches the
			// origin. EPA must resolve the (zero) penetration. Keep the
			// two points so EPA can grow a triangle from them.
			return true, s, Vec2{}
		}
		return false, s, d
	}
}

// reorderCCW returns the three support points so that their V points are
// ordered counter-clockwise.
func reorderCCW(s []SupportPoint) []SupportPoint {
	area := s[1].V.Sub(s[0].V).Cross(s[2].V.Sub(s[0].V))
	if area >= 0 {
		return []SupportPoint{s[0], s[1], s[2]}
	}
	// Swap the last two to flip orientation.
	return []SupportPoint{s[0], s[2], s[1]}
}

// evolveTriangle performs one GJK update for a three-vertex simplex whose
// points are reordered to CCW. It returns the reduced simplex, the next
// search direction and whether the origin is contained.
func evolveTriangle(s []SupportPoint, tol float64) (contained bool, out []SupportPoint, dir Vec2) {
	t := reorderCCW(s)
	// For a CCW triangle the interior lies to the left of every directed
	// edge p_i -> p_j; its inward normal is edge.PerpLeft().
	type edge struct {
		i, j int
		sp   [2]SupportPoint
	}
	edges := [3]edge{
		{i: 0, j: 1, sp: [2]SupportPoint{t[0], t[1]}},
		{i: 1, j: 2, sp: [2]SupportPoint{t[1], t[2]}},
		{i: 2, j: 0, sp: [2]SupportPoint{t[2], t[0]}},
	}

	bestOutside := 0.0
	bestEdge := -1
	var bestNormal Vec2
	inside := true
	for _, e := range edges {
		evec := t[e.j].V.Sub(t[e.i].V)
		if evec.Len2() <= tol*tol {
			continue
		}
		nIn := evec.PerpLeft().Normalized()
		// Signed distance of the origin past this edge: nIn·p > 0 means
		// the origin lies beyond the edge (outside the triangle), since
		// the left normal of a CCW directed edge points inward.
		outDist := nIn.Dot(t[e.i].V)
		if outDist > tol {
			inside = false
			if outDist > bestOutside {
				bestOutside = outDist
				bestEdge = e.i
				bestNormal = nIn.Scale(-1)
			}
		}
	}
	if inside {
		return true, t, Vec2{}
	}
	e := edges[bestEdge]
	// Search outward across that edge; keep the edge as a segment simplex.
	return false, []SupportPoint{e.sp[1], e.sp[0]}, bestNormal
}

// closestLineFeature reduces a degenerate (collinear) triangle to the segment
// or vertex closest to the origin and gives the direction toward the origin.
func closestLineFeature(s []SupportPoint, tol float64) (out []SupportPoint, dir Vec2) {
	bestD := math.Inf(1)
	var bestQ Vec2
	var bestP [2]SupportPoint
	var bestT float64
	pairs := [][2]SupportPoint{{s[0], s[1]}, {s[1], s[2]}, {s[2], s[0]}}
	for _, pr := range pairs {
		ab := pr[1].V.Sub(pr[0].V)
		if ab.Len2() <= tol*tol {
			if d := pr[0].V.Len(); d < bestD {
				bestD = d
				bestQ = pr[0].V
				bestP = pr
				bestT = 0
			}
			continue
		}
		t := clampUnit(pr[0].V.Scale(-1).Dot(ab) / ab.Len2())
		q := pr[0].V.Add(ab.Scale(t))
		if d := q.Len(); d < bestD {
			bestD = d
			bestQ = q
			bestP = pr
			bestT = t
		}
	}
	if bestT <= paramTol || bestT >= 1-paramTol {
		sp := bestP[0]
		if bestT >= 1-paramTol {
			sp = bestP[1]
		}
		return []SupportPoint{sp}, sp.V.Scale(-1)
	}
	return []SupportPoint{bestP[1], bestP[0]}, bestQ.Scale(-1)
}

func clampUnit(t float64) float64 {
	switch {
	case t < 0:
		return 0
	case t > 1:
		return 1
	default:
		return t
	}
}
