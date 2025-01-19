(define (domain replan)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
piece
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(robot_from ?r - robot ?wp - waypoint)
(connected ?wp_from ?wp_to - waypoint)
(piece_at ?p - piece ?wp - waypoint) 
(piece_at_robot ?r - robot ?p - piece)


);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

(nav_time ?from ?to - waypoint)

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration (* (nav_time ?from ?to) 5))
    :condition (and
        (at start(robot_at ?r ?from))
        (over all(connected ?from ?to))
        )
    :effect (and
        (at start(robot_from ?r ?from))
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
        (at end(not(robot_from ?r ?from)))
    )
)

(:durative-action pick
    :parameters (?r - robot ?p - piece ?at - waypoint)
    :duration ( = ?duration 1)
    :condition (and
        (over all(robot_at ?r ?at))
        (at start(piece_at ?p ?at))
    )
    :effect (and
        (at start(not(piece_at ?p ?at)))
        (at end(piece_at_robot ?r ?p))
    )
)

(:durative-action place
    :parameters (?r - robot ?p - piece ?at - waypoint)
    :duration ( = ?duration 1)
    :condition (and
        (over all(robot_at ?r ?at))
        (at start(piece_at_robot ?r ?p))
    )
    :effect (and
        (at end(not(piece_at_robot ?r ?p)))
        (at end(piece_at ?p ?at))
    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
