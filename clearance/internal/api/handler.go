// Package api exposes the clearance kernel over HTTP (Gin).
package api

import (
	"errors"
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"

	"clearance/internal/geometry"
)

// pointDTO is one polygon vertex in a request.
type pointDTO struct {
	X *float64 `json:"x"`
	Y *float64 `json:"y"`
}

// collideRequest is the query body for POST /collide.
type collideRequest struct {
	PolygonA []pointDTO `json:"polygonA"`
	PolygonB []pointDTO `json:"polygonB"`
}

// errorResponse is the uniform error body.
type errorResponse struct {
	Code    string `json:"code"`
	Message string `json:"message"`
}

// Router builds the HTTP handler tree.
func Router() *gin.Engine {
	r := gin.New()
	r.Use(gin.Recovery())
	r.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{"status": "ok"})
	})
	r.POST("/collide", handleCollide)
	return r
}

func handleCollide(c *gin.Context) {
	var req collideRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, errorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygonA and polygonB vertex arrays: " + err.Error(),
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

	res, err := geometry.Evaluate(aPts, bPts)
	if err != nil {
		writeKernelError(c, err)
		return
	}
	c.JSON(http.StatusOK, res)
}

func toPoints(which string, dto []pointDTO) ([]geometry.Vec2, error) {
	if len(dto) == 0 {
		return nil, &geometry.KernelError{
			Code:    geometry.ErrTooFewVertices,
			Message: "polygon " + which + ": missing or empty vertex array",
		}
	}
	pts := make([]geometry.Vec2, len(dto))
	for i, p := range dto {
		if p.X == nil || p.Y == nil {
			return nil, &geometry.KernelError{
				Code:    geometry.ErrNonFiniteCoordinate,
				Message: "polygon " + which + ": vertex " + strconv.Itoa(i) + " must have numeric x and y",
			}
		}
		pts[i] = geometry.Vec2{X: *p.X, Y: *p.Y}
	}
	return pts, nil
}

func writeKernelError(c *gin.Context, err error) {
	var ke *geometry.KernelError
	if errors.As(err, &ke) {
		status := http.StatusBadRequest
		if ke.Code == geometry.ErrGJKNoConvergence || ke.Code == geometry.ErrEPANoConvergence ||
			ke.Code == geometry.ErrEPAFailure {
			status = http.StatusUnprocessableEntity
		}
		c.JSON(status, errorResponse{Code: ke.Code, Message: ke.Message})
		return
	}
	c.JSON(http.StatusInternalServerError, errorResponse{Code: "INTERNAL", Message: err.Error()})
}
