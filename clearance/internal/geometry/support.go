package geometry

// SupportPoint is one vertex of the Minkowski difference A⊖B together with the
// two input vertices ("witness" points) it was built from.
type SupportPoint struct {
	// V is the point a - b on the Minkowski difference.
	V Vec2
	// Pa is the supporting vertex of A.
	Pa Vec2
	// Pb is the supporting vertex of B (negated inside V).
	Pb Vec2
}

// furthestVertex returns the vertex of p with the largest projection on d.
// Ties resolve deterministically to the first maximizing vertex.
func furthestVertex(p *Polygon, d Vec2) (int, Vec2) {
	bestI := 0
	best := p.Vertices[0].Dot(d)
	for i := 1; i < len(p.Vertices); i++ {
		if v := p.Vertices[i].Dot(d); v > best {
			best = v
			bestI = i
		}
	}
	return bestI, p.Vertices[bestI]
}

// support evaluates the support function of the Minkowski difference A⊖B
// along d: h_A(d) - h_B(-d). It must be called with a non-zero direction.
func support(a, b *Polygon, d Vec2) SupportPoint {
	_, pa := furthestVertex(a, d)
	_, pb := furthestVertex(b, d.Scale(-1))
	return SupportPoint{V: pa.Sub(pb), Pa: pa, Pb: pb}
}
