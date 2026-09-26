package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"clearance/internal/geometry"
)

// velocityDTO is one constant-velocity vector in a sweep request. A missing
// velocity object means the part does not move during the window.
type velocityDTO struct {
	X *float64 `json:"x"`
	Y *float64 `json:"y"`
}

// sweepRequest is the query body for POST /sweep: the two contours at t=0
// and their constant translation velocities over the window t in [0,1].
type sweepRequest struct {
	PolygonA  []pointDTO   `json:"polygonA"`
	PolygonB  []pointDTO   `json:"polygonB"`
	VelocityA *velocityDTO `json:"velocityA"`
	VelocityB *velocityDTO `json:"velocityB"`
}

// handleSweep serves POST /sweep, the continuous (time-aware) collision
// query. It is a separate entry point from POST /collide: the static query
// is untouched and clients of the old endpoint are unaffected.
func handleSweep(c *gin.Context) {
	var req sweepRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygonA, polygonB vertex arrays and optional velocityA/velocityB vectors: " + err.Error(),
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
	va, err := toVelocity("A", req.VelocityA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	vb, err := toVelocity("B", req.VelocityB)
	if err != nil {
		writeKernelError(c, err)
		return
	}

	res, err := geometry.Sweep(aPts, bPts, va, vb)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	c.JSON(http.StatusOK, res)
}

// toVelocity converts the optional velocity DTO. A missing velocity means a
// stationary part; a present velocity must carry numeric x and y.
func toVelocity(which string, dto *velocityDTO) (geometry.Vec2, error) {
	if dto == nil {
		return geometry.Vec2{}, nil
	}
	if dto.X == nil || dto.Y == nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    geometry.ErrNonFiniteCoordinate,
			Message: "velocity " + which + ": must have numeric x and y",
		}
	}
	return geometry.Vec2{X: *dto.X, Y: *dto.Y}, nil
}
