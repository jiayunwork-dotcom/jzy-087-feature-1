// Command clearance-svc serves the assembly-clearance geometric kernel over
// HTTP. It exposes no UI: POST /collide answers the static single-frame query
// for two convex polygons and POST /sweep answers the constant-velocity
// swept query (time of impact / closest approach over t in [0,1]).
package main

import (
	"log"
	"os"

	"clearance/internal/api"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	r := api.Router()
	log.Printf("clearance service listening on :%s", port)
	if err := r.Run(":" + port); err != nil {
		log.Fatalf("server failed: %v", err)
	}
}
