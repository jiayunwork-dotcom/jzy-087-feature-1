package geometry

// separation is the separated-branch result: the closest feature of the
// Minkowski difference to the origin, expressed back on the two polygons.
type separation struct {
	// distance is the minimum gap, i.e. the Euclidean distance from the
	// origin to A⊖B.
	distance float64
	// normal is the unit direction from the closest point on A toward the
	// closest point on B (the contact/gap normal).
	normal Vec2
	// pointA / pointB are the closest point pair on the two polygons.
	pointA, pointB Vec2
}

// closestOnSimplex extracts the closest points from a final separated GJK
// simplex of one or two support points. A two-point simplex stores the edge
// with the newest point first; the interpolation formula is order-free.
func closestOnSimplex(s []SupportPoint, tol float64) separation {
	if len(s) == 1 {
		d := s[0].V.Len()
		// The vector from the closest CSO point to the origin is -v, which
		// is precisely the direction from the witness point on A toward the
		// witness point on B.
		n := s[0].V.Scale(-1).Normalized()
		// Closest points are the witness vertices themselves.
		return separation{
			distance: d,
			normal:   n,
			pointA:   s[0].Pa,
			pointB:   s[0].Pb,
		}
	}

	p1, p2 := s[0], s[1]
	e := p2.V.Sub(p1.V)
	t := 0.0
	if e.Len2() > tol*tol {
		t = clampUnit(p1.V.Scale(-1).Dot(e) / e.Len2())
	}
	q := p1.V.Add(e.Scale(t))
	pa := p1.Pa.Add(p2.Pa.Sub(p1.Pa).Scale(t))
	pb := p1.Pb.Add(p2.Pb.Sub(p1.Pb).Scale(t))
	return separation{
		distance: q.Len(),
		normal:   q.Scale(-1).Normalized(),
		pointA:   pa,
		pointB:   pb,
	}
}
