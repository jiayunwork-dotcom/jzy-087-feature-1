package geometry

import "math"

// MaxGJKIterations bounds the number of simplex evolution steps. A genuine
// GJK run on a polygon terminates in a handful of steps; exhausting this
// budget is reported as an error instead of looping forever.
const MaxGJKIterations = 128

// gjkMaxSteps is the live bound, indirected so tests can force exhaustion.
var gjkMaxSteps = MaxGJKIterations

type gjkState struct {
	// contained is true when the simplex reaches the origin (origin inside
	// A⊖B ⇔ the polygons overlap).
	contained bool
	// simplex is the final simplex:
	//   contained=true  -> 1..3 support points seeding EPA,
	//   contained=false -> 1..2 support points, the closest feature found.
	simplex []SupportPoint
	steps   int
}

// gjk runs the simplex iteration on the Minkowski difference A⊖B, driven by
// support evaluations, and decides whether the origin belongs to the CSO.
func gjk(a, b *Polygon) (*gjkState, error) {
	tol := epsFor(a.maxLen(), b.maxLen())

	d := a.centroid().Sub(b.centroid())
	if d.Len2() == 0 {
		d = Vec2{X: 1}
	}
	s0 := support(a, b, d)
	simplex := []SupportPoint{s0}
	d = s0.V.Scale(-1)

	for step := 1; step <= gjkMaxSteps; step++ {
		if d.Len() <= tol {
			return &gjkState{contained: true, simplex: simplex, steps: step}, nil
		}
		sp := support(a, b, d)

		// The support point must make progress toward the origin along d.
		// If it is already part of the simplex, the closest feature has
		// been reached and the origin stays outside.
		if duplicatesAny(sp, simplex, tol) {
			if sp.V.Len() <= tol {
				return &gjkState{contained: true, simplex: simplex, steps: step}, nil
			}
			return &gjkState{contained: false, simplex: simplex, steps: step}, nil
		}
		simplex = append([]SupportPoint{sp}, simplex...)

		switch len(simplex) {
		case 2:
			contained, out, nd := evolveLine(simplex, tol)
			simplex, d = out, nd
			if contained {
				return &gjkState{contained: true, simplex: simplex, steps: step}, nil
			}
		case 3:
			area := simplex[1].V.Sub(simplex[0].V).Cross(simplex[2].V.Sub(simplex[0].V))
			if math.Abs(area) <= tol*tol {
				// Numerically degenerate triangle: reduce to the closest
				// segment/vertex feature and keep iterating (or detect
				// boundary contact).
				out, nd := closestLineFeature(simplex, tol)
				simplex, d = out, nd
				if d.Len() <= tol {
					return &gjkState{contained: true, simplex: simplex, steps: step}, nil
				}
				continue
			}
			contained, out, nd := evolveTriangle(simplex, tol)
			simplex, d = out, nd
			if contained {
				return &gjkState{contained: true, simplex: simplex, steps: step}, nil
			}
		}
	}
	return nil, &KernelError{Code: ErrGJKNoConvergence,
		Message: "GJK simplex iteration did not converge within the step budget"}
}

func duplicatesAny(sp SupportPoint, simplex []SupportPoint, tol float64) bool {
	for _, s := range simplex {
		if s.V.Sub(sp.V).Len() <= tol {
			return true
		}
	}
	return false
}
