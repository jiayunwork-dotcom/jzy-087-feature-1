package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"clearance/internal/geometry"
)

// sweepRequest is the query body for POST /sweep: two convex polygons with
// their initial poses and constant translation velocities over t in [0,1].
type sweepRequest struct {
	PolygonA []pointDTO `json:"polygonA"`
	PolygonB []pointDTO `json:"polygonB"`
	// OffsetA / OffsetB optionally translate the given vertices into the
	// initial pose (defaults to zero).
	OffsetA *pointDTO `json:"offsetA"`
	OffsetB *pointDTO `json:"offsetB"`
	// VelocityA / VelocityB are the constant per-unit-time translation
	// velocities (defaults to zero).
	VelocityA *pointDTO `json:"velocityA"`
	VelocityB *pointDTO `json:"velocityB"`
}

// handleSweep serves POST /sweep, the time-dependent clearance query. It is
// deliberately separate from the static /collide handler: the static
// request/response contract is untouched.
func handleSweep(c *gin.Context) {
	var req sweepRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygonA/polygonB vertex arrays and optional offsetA/offsetB/velocityA/velocityB vectors: " + err.Error(),
		})
		return
	}

	aPts, err := toPoints("A", req.PolygonA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	bPts, err := toPoints("B", req.PolygonB)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	offA, err := toVector("offsetA", req.OffsetA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	offB, err := toVector("offsetB", req.OffsetB)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	velA, err := toVector("velocityA", req.VelocityA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	velB, err := toVector("velocityB", req.VelocityB)
	if err != nil {
		writeKernelError(c, err)
		return
	}

	// Initial pose = given vertices shifted by the optional offset.
	for i := range aPts {
		aPts[i] = aPts[i].Add(offA)
	}
	for i := range bPts {
		bPts[i] = bPts[i].Add(offB)
	}

	res, err := geometry.Sweep(aPts, bPts, velA, velB)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	c.JSON(http.StatusOK, res)
}

// toVector converts an optional 2D vector DTO (offset/velocity); a missing
// vector means zero. A present vector with a missing component is rejected.
func toVector(name string, p *pointDTO) (geometry.Vec2, error) {
	if p == nil {
		return geometry.Vec2{}, nil
	}
	if p.X == nil || p.Y == nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    geometry.ErrNonFiniteCoordinate,
			Message: name + ": must have numeric x and y",
		}
	}
	return geometry.Vec2{X: *p.X, Y: *p.Y}, nil
}
