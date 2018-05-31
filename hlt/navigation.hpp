#pragma once

#include "collision.hpp"
#include "map.hpp"
#include "move.hpp"
#include "util.hpp"

namespace hlt {
    namespace navigation {
        // Method checks if there's a ship in the way of another ship
        static bool check_moving_ship_between(const Location& start,
                    const Location &target,const Move &mv, const hlt::Map &map,
                    const Entity &ship2) {
            // Get the initial ship's position
            double ship1_x = start.pos_x;
            double ship1_y = start.pos_y;
            // Get the secondary ship's position
            double ship2_x = ship2.location.pos_x;
            double ship2_y = ship2.location.pos_y;
            // Consider the first ship's target position
            double dest1_x = target.pos_x;
            double dest1_y = target.pos_y;

            double dest2_x = 0.0;
            double dest2_y = 0.0;

            // Used for determining the movement of the second ship
            int move2_thrust = mv.move_thrust;
            int angle2 = mv.move_angle_deg;

            // Consider slopes
            double slope1 = (ship1_y - dest1_y) / (ship1_x - dest1_x);
            double slope2 = tan(angle2);

            // y = m * x + n => freeTerm1/freeTerm2 = n; slope1/slope2 = m;
            double freeTerm1 = dest1_y - dest1_x * ((ship1_y - dest1_y)
            / (ship1_x - dest1_x));
            double freeTerm2 = (-1) * slope2 * ship2_x + ship2_y;

            // y = m * x + n, y1 -yo = m(x1 - x0) => 2nd degree ecuation
            // that gives the destination coordinates needed
            double a = 1 + slope2 * slope2;
            double b = (-2) * ship2_x - 2 * slope2 * slope2 * ship2_x;
            double c = (slope2 * slope2 + 1)
            * ship2_x * ship2_x - move2_thrust;
            // 2nd degree ecuation has 2 results, consider both initially
            double dest2_x_1 = 0.0;
            double dest2_y_1 = 0.0;
            double dest2_x_2 = 0.0;
            double dest2_y_2 = 0.0;

            // Determine the solutions
            dest2_x_1 = (-b - sqrt(b * b - 4 * a * c) ) / (2 * a);
            dest2_x_2 = (-b + sqrt(b * b - 4 * a * c) ) / (2 * a);

            dest2_y_1 = dest2_x_1 * slope2 + freeTerm2;
            dest2_y_2 = dest2_x_2 * slope2 + freeTerm2;

            // Choose the right solution in accordance to the map's
            // xOy and the angle consensus
            if (angle2 >= 0 && angle2 < 180) {
                if (dest2_y_1 > ship2_y) {
                    dest2_x = dest2_x_1;
                    dest2_y = dest2_y_1;
                } else {
                    dest2_x = dest2_x_2;
                    dest2_y = dest2_y_2;
                }
            } else {
                if (dest2_y_1 < ship1_y) {
                    dest2_x = dest2_x_1;
                    dest2_y = dest2_y_1;
                } else {
                    dest2_x = dest2_x_2;
                    dest2_y = dest2_y_2;
                }
            }
            // Determine the intersection coordinates
            double inter_x = (freeTerm2 - freeTerm1) / (slope1 - slope2);
            double inter_y = slope1 * inter_x + freeTerm1;

            // "Plant" a ship right in the intersection point, ship is passive
            Entity aux_ship;
            aux_ship.location.pos_x = inter_x;
            aux_ship.location.pos_y = inter_y;
            aux_ship.radius = constants::SHIP_RADIUS;

            Location start_2;
            start_2.pos_x = ship2_x;
            start_2.pos_y = ship2_y;

            Location target_2;
            target_2.pos_x = dest2_x;
            target_2.pos_y = dest2_y;


            // If there's going to be a collision from both real ships
            // with the fake ship, then a real collision might happen
            // avoid it.
            if (collision::segment_circle_intersect(start, target,
                aux_ship, constants::FORECAST_FUDGE_FACTOR)
                && collision::segment_circle_intersect(start_2, target_2,
                    aux_ship, constants::FORECAST_FUDGE_FACTOR)) {

                    return true;
            }

            return false;
        }


        static bool check_standing_object_between(const Location& start,
                                                const Location& target,
                                                const Entity& entity_to_check) {

            const Location &location = entity_to_check.location;
            if (location == start || location == target) {
                return false;
            }
            if (collision::segment_circle_intersect(start, target,
                entity_to_check, constants::FORECAST_FUDGE_FACTOR)) {

                return true;
            }
            return false;
        }

        static bool objects_between(const Map& map, const Location& start,
                    const Location& target, std::vector<hlt::Move> moves) {

            // Verifies if course intersects with planets
            for (const Planet& planet : map.planets) {
                if (check_standing_object_between(start, target, planet)) {
                    return true;
                }
            }

            // Verifies if course intersects with other ships
            for (const auto& player_ship : map.ships) {
                for (const Ship& ship : player_ship.second) {
                    bool found_moving = false;
                    // Finds out if that ship is moving or not so the suitable
                    // method is applied
                    for (const auto &move : moves) {
                        if (move.ship_id == ship.entity_id
                            && move.type == MoveType::Thrust) {

                            found_moving = true;

                            if (check_moving_ship_between(start, target,
                                move, map, ship)) {

                                 return true;
                            }
                            break;
                        }
                    }
                    if (!found_moving && check_standing_object_between(start,
                    target, ship)) {
                        return true;
                    }
                }
            }
            return false;
        }

        static possibly<Move> navigate_ship_towards_target(
                const Map& map, const Ship& ship, const Location& target,
                const int max_thrust, const bool avoid_obstacles,
                const int max_corrections, const double angular_step_rad,
                std::vector<Move> moves) {

            if (max_corrections <= 0) {
                return { Move::noop(), false };
            }

            const double distance = ship.location.get_distance_to(target);
            const double ang_rad = ship.location.orient_towards_in_rad(target);

            // Verifies if ship has objects between it's starting point and
            // target. If so, it alters it's course by one degree until the
            // path is clear or no trajectory can be used.
            if (avoid_obstacles
                && objects_between(map, ship.location, target, moves)) {

                const double new_target_dx =
                cos(ang_rad + angular_step_rad) * distance;

                const double new_target_dy =
                sin(ang_rad + angular_step_rad) * distance;

                const Location new_target = { ship.location.pos_x +
                    new_target_dx, ship.location.pos_y + new_target_dy };

                return navigate_ship_towards_target(map, ship, new_target,
                       max_thrust, true, (max_corrections - 1),
                       angular_step_rad, moves);
            }

            int thrust;
            if (distance < max_thrust) {
                // Do not round up, since overshooting might cause collision.
                thrust = (int) distance;
            } else {
                thrust = max_thrust;
            }

            const int angle_deg = util::angle_rad_to_deg_clipped(ang_rad);

            return { Move::thrust(ship.entity_id, thrust, angle_deg), true };
        }

        static possibly<Move> navigate_ship_to_dock(
                const Map& map, const Ship& ship, const Entity& dock_target,
                const int max_thrust, std::vector<Move> moves) {

            const int max_corrections = constants::MAX_NAVIGATION_CORRECTIONS;
            const bool avoid_obstacles = true;
            const double angular_step_rad = M_PI / 180.0;
            const Location& target =
            ship.location.get_closest_point(dock_target.location,
                                            dock_target.radius);

            return navigate_ship_towards_target(map, ship, target, max_thrust,
                avoid_obstacles, max_corrections, angular_step_rad, moves);
        }
    }
}
