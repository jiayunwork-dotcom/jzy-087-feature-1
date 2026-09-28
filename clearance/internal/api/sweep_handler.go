package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"clearance/internal/geometry"
)

// sweepRequest is the query body for POST /sweep. Each contour is given in
// the part's local frame; offsetA/offsetB (optional, default zero) place the
// local frames at t=0 and velocityA/velocityB are the constant translations
// over t in [0,1].
type sweepRequest struct {
	PolygonA  []pointDTO `json:"polygonA"`
	PolygonB  []pointDTO `json:"polygonB"`
	OffsetA   *pointDTO  `json:"offsetA"`
	OffsetB   *pointDTO  `json:"offsetB"`
	VelocityA *pointDTO  `json:"velocityA"`
	VelocityB *pointDTO  `json:"velocityB"`
}

func handleSweep(c *gin.Context) {
	var req sweepRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygonA/polygonB vertex arrays and velocityA/velocityB vectors: " + err.Error(),
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
	oa, err := toOffset("A", req.OffsetA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	ob, err := toOffset("B", req.OffsetB)
	if err != nil {
		writeKernelError(c, err)
		return
	}

	res, err := geometry.Sweep(
		geometry.Motion{Vertices: aPts, Offset: oa, Velocity: va},
		geometry.Motion{Vertices: bPts, Offset: ob, Velocity: vb},
	)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	c.JSON(http.StatusOK, res)
}

// toVelocity requires an explicit vector with two numeric components.
func toVelocity(which string, dto *pointDTO) (geometry.Vec2, error) {
	if dto == nil || dto.X == nil || dto.Y == nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    geometry.ErrNonFiniteVelocity,
			Message: "polygon " + which + ": velocity must be an {x,y} vector with numeric components",
		}
	}
	return geometry.Vec2{X: *dto.X, Y: *dto.Y}, nil
}

// toOffset accepts an omitted offset as zero; when present it must be numeric.
func toOffset(which string, dto *pointDTO) (geometry.Vec2, error) {
	if dto == nil {
		return geometry.Vec2{}, nil
	}
	if dto.X == nil || dto.Y == nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    geometry.ErrNonFiniteCoordinate,
			Message: "polygon " + which + ": initial offset must have numeric x and y",
		}
	}
	return geometry.Vec2{X: *dto.X, Y: *dto.Y}, nil
}
