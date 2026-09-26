package geometry

// Status values reported to clients.
const (
	StatusSeparated  = "separated"
	StatusPenetrated = "penetrated"
)

// Result is the kernel answer for a pair of convex polygons.
type Result struct {
	// Status is StatusSeparated or StatusPenetrated.
	Status string `json:"status"`
	// Distance is the minimum clearance when separated (0 when penetrated).
	Distance float64 `json:"distance"`
	// PenetrationDepth is how far the parts overlap (0 when separated).
	PenetrationDepth float64 `json:"penetrationDepth"`
	// Normal is the contact normal:
	//   separated  -> unit vector from the closest point on A to that on B,
	//   penetrated -> direction in which B must be translated to just
	//                 separate the parts (outward normal of A⊖B).
	Normal Vec2 `json:"normal"`
	// PointA / PointB are the closest or contact points on the two parts.
	PointA Vec2 `json:"pointA"`
	PointB Vec2 `json:"pointB"`
	// Iterations is the total number of simplex/expansion steps consumed.
	Iterations int `json:"iterations"`
}

// Evaluate validates the two contours and runs the Minkowski-difference
// simplex evaluation.
func Evaluate(aPts, bPts []Vec2) (*Result, error) {
	a, err := NewPolygon(aPts)
	if err != nil {
		return nil, namedError("A", err.(*KernelError))
	}
	b, err := NewPolygon(bPts)
	if err != nil {
		return nil, namedError("B", err.(*KernelError))
	}

	st, err := gjk(a, b)
	if err != nil {
		return nil, err
	}

	if !st.contained {
		// Origin outside A⊖B: the bodies are separated. The distance from
		// the origin to the simplex equals the gap; witnesses give the
		// closest point pair.
		tol := epsFor(a.maxLen(), b.maxLen())
		sep := closestOnSimplex(st.simplex, tol)
		if sep.distance <= tol {
			// Defensive guard: never report penetration as a zero gap
			// "separated" result. Route boundary cases through EPA.
			return evaluateContact(a, b, st.simplex, st.steps)
		}
		return &Result{
			Status:     StatusSeparated,
			Distance:   sep.distance,
			Normal:     sep.normal.cleanZeros(),
			PointA:     sep.pointA.cleanZeros(),
			PointB:     sep.pointB.cleanZeros(),
			Iterations: st.steps,
		}, nil
	}
	return evaluateContact(a, b, st.simplex, st.steps)
}

// evaluateContact runs EPA for an origin-containing simplex.
func evaluateContact(a, b *Polygon, seed []SupportPoint, gjkSteps int) (*Result, error) {
	c, err := epa(a, b, seed, gjkSteps)
	if err != nil {
		return nil, err
	}
	return &Result{
		Status:           StatusPenetrated,
		PenetrationDepth: c.depth,
		Normal:           c.normal.cleanZeros(),
		PointA:           c.pointA.cleanZeros(),
		PointB:           c.pointB.cleanZeros(),
		Iterations:       c.steps,
	}, nil
}
