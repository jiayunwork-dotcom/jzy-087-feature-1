package api

import (
	"net/http"

	"github.com/gin-gonic/gin"

	"clearance/internal/geometry"
)

// vec2DTO is an optional 2-D vector in a request. A nil pointer means the
// field was absent and defaults to the zero vector.
type vec2DTO struct {
	X *float64 `json:"x"`
	Y *float64 `json:"y"`
}

// sweepRequest is the query body for POST /sweep.
//
// Each part is described by its contour (polygonX), an optional initial pose
// offset (poseX; "offsetX" is accepted as an alias) and a constant
// translation velocity (velocityX). Over [0,1] part X occupies
// polygonX + poseX + t*velocityX.
type sweepRequest struct {
	PolygonA  []pointDTO `json:"polygonA"`
	PolygonB  []pointDTO `json:"polygonB"`
	PoseA     *vec2DTO   `json:"poseA"`
	PoseB     *vec2DTO   `json:"poseB"`
	OffsetA   *vec2DTO   `json:"offsetA"`
	OffsetB   *vec2DTO   `json:"offsetB"`
	VelocityA *vec2DTO   `json:"velocityA"`
	VelocityB *vec2DTO   `json:"velocityB"`
}

// handleSweep serves POST /sweep, the continuous swept query. It is kept
// fully separate from handleCollide: the static endpoint, its request and
// its response are unchanged.
func handleSweep(c *gin.Context) {
	var req sweepRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygonA and polygonB vertex arrays and optional pose/velocity vectors: " + err.Error(),
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

	poseA, err := optionalVec("pose A", req.PoseA, req.OffsetA)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	poseB, err := optionalVec("pose B", req.PoseB, req.OffsetB)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	velA, err := optionalVec("velocity A", req.VelocityA, nil)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	velB, err := optionalVec("velocity B", req.VelocityB, nil)
	if err != nil {
		writeKernelError(c, err)
		return
	}

	res, err := geometry.Swept(geometry.SweptInput{
		A:         aPts,
		B:         bPts,
		OffsetA:   poseA,
		OffsetB:   poseB,
		VelocityA: velA,
		VelocityB: velB,
	})
	if err != nil {
		writeKernelError(c, err)
		return
	}
	c.JSON(http.StatusOK, res)
}

// optionalVec decodes a pose/velocity vector. primary (preferred name) and
// alias are mutually exclusive JSON fields; an absent vector is the zero
// vector. A vector with a non-numeric component is rejected at the API layer
// with a readable message (the kernel additionally guards NaN/Inf).
func optionalVec(which string, primary, alias *vec2DTO) (geometry.Vec2, error) {
	if primary != nil && alias != nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    "INVALID_JSON",
			Message: which + ": provide the vector once (do not set both accepted field names)",
		}
	}
	v := primary
	if v == nil {
		v = alias
	}
	if v == nil {
		return geometry.Vec2{}, nil
	}
	if v.X == nil || v.Y == nil {
		return geometry.Vec2{}, &geometry.KernelError{
			Code:    geometry.ErrNonFiniteCoordinate,
			Message: which + ": vector must have numeric x and y components",
		}
	}
	return geometry.Vec2{X: *v.X, Y: *v.Y}, nil
}
