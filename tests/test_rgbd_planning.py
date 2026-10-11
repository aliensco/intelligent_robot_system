"""Follow endpoint selection must respect footprint and connectivity."""
import math
import os
import sys
import unittest
import json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.motion import Grid
from rgbd_following_core.planning import reachable_standoff, _approach_angle


class PlanningTests(unittest.TestCase):
    def test_sideways_turns_cannot_cancel_in_a_forward_approach_chord(self):
        # A dogleg corridor turns out and back before advancing. Its position
        # after 25 cm is directly ahead, concealing both lateral turns from
        # the old whole-prefix chord score. A gentle alternative should win.
        gentle = [(0., 0.), (.30, .10)]
        gentle_angle = _approach_angle(gentle, (2., 0.), .30)
        for side in (-1., 1.):
            dogleg = [(0., 0.), (0., .10*side), (.04, .10*side),
                      (.04, 0.), (.34, 0.)]
            angle = _approach_angle(dogleg, (2., 0.), .30)
            self.assertGreater(angle, math.radians(85.))
            self.assertGreater(angle, gentle_angle)
        self.assertLess(gentle_angle, math.radians(25.))

    def test_approach_uses_target_bearing_at_the_local_segment(self):
        path = [(0., 0.), (.10, .10), (.20, .20), (.30, .30)]
        # Both targets start straight ahead, but translation shifts the nearer
        # one toward the image edge much faster. Initial bearing alone misses it.
        near = _approach_angle(path, (.60, 0.), .45)
        far = _approach_angle(path, (3., 0.), .45)
        self.assertGreater(near, far+math.radians(25.))
        self.assertGreater(near, math.radians(75.))

    def test_approach_cost_is_limited_to_the_subgoal_horizon(self):
        path = [(0., 0.), (.40, 0.), (.40, .50)]
        self.assertAlmostEqual(_approach_angle(path, (2., 0.), .30), 0.)
        self.assertGreater(_approach_angle(path, (2., 0.), .60), math.radians(80.))
        self.assertAlmostEqual(_approach_angle([(0., 0.), (.04, 0.)], (2., 0.), .30), 0.)
        self.assertAlmostEqual(_approach_angle([(0., 0.), (0., 0.)], (2., 0.), .30), 0.)

    def test_approach_cost_does_not_depend_on_world_frame_heading(self):
        path, target = [(0., 0.), (.10, .08), (.25, .10)], (1., .05)
        angle = _approach_angle(path, target, .30)
        c, s = math.cos(1.2), math.sin(1.2)
        def transform(point):
            return 2.+c*point[0]-s*point[1], -3.+s*point[0]+c*point[1]
        self.assertAlmostEqual(_approach_angle([transform(p) for p in path],
                                               transform(target), .30), angle)

    def test_recorded_cone_failure_has_reachable_detour(self):
        with open(os.path.join(os.path.dirname(__file__), 'fixtures/rgbd_cone_standoff.json')) as source:
            fixture = json.load(source)
        cfg = fixture['grid']
        cfg['data'] = [value for value,count in cfg.pop('runs') for unused in range(count)]
        g = Grid(**cfg)
        from rgbd_following_core.motion import clear_standoff
        self.assertIsNone(clear_standoff(g,fixture['robot'],fixture['target'],.6,.26,.5))
        point,detour = reachable_standoff(g,fixture['robot'],fixture['target'],.6,.5,.26,.6)
        self.assertTrue(detour)
        self.assertTrue(g.clear(point[0],point[1],.26))

    def grid(self):
        return Grid([0]*40000, 200, 200, .02, (-2., -2., 0.))

    def paint(self, grid, predicate, value=100):
        for iy in range(grid.height):
            for ix in range(grid.width):
                x = grid.origin[0]+(ix+.5)*grid.resolution
                y = grid.origin[1]+(iy+.5)*grid.resolution
                if predicate(x, y):
                    grid.data[iy*grid.width+ix] = value

    def test_normal_gap_is_preferred_on_open_map(self):
        g = self.grid()
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6, path_lookahead=0.)
        self.assertFalse(detour)
        self.assertAlmostEqual(math.hypot(point[0]-1.2,point[1]), .6)

    def test_blocked_distance_ring_uses_farther_reachable_waypoint(self):
        g = self.grid()
        for iy in range(g.height):
            for ix in range(g.width):
                x,y = -2+(ix+.5)*.02,-2+(iy+.5)*.02
                if math.hypot(x-1.2,y) < .6:
                    g.data[iy*g.width+ix] = 100
        result = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6)
        self.assertIsNotNone(result)
        point,detour = result
        self.assertTrue(detour)
        self.assertTrue(g.clear(point[0],point[1],.12))
        self.assertGreater(math.hypot(point[0]-1.2,point[1]), .6)
        self.assertLess(math.hypot(point[0]-1.2,point[1]), 1.2)

    def test_clear_but_disconnected_endpoint_is_rejected(self):
        for obstacle in (100, -1):
            g = self.grid()
            for iy in range(g.height):
                g.data[iy*g.width+117] = obstacle
            self.assertIsNone(reachable_standoff(g,(0,0,0),(1.2,0),.6,.5,.12,.1))

    def test_failed_goal_is_not_immediately_selected_again(self):
        point,detour = reachable_standoff(self.grid(),(0,0,0),(1.2,0),.6,.5,.12,.6, (.6,0,0))
        self.assertGreaterEqual(math.hypot(point[0]-.6,point[1]), .15)

    def test_multiple_failed_or_visited_goals_are_all_excluded(self):
        g = self.grid()
        visited = []
        for unused in range(4):
            point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6, visited)
            for previous in visited:
                self.assertGreaterEqual(math.hypot(point[0]-previous[0], point[1]-previous[1]), .15)
            visited.append(point)

    def test_blocked_start_cannot_be_relabelled_free(self):
        g=self.grid();g.data[100*g.width+100]=100
        self.assertIsNone(reachable_standoff(g,(0,0,0),(1.2,0),.6,.5,.12,.6))

    def test_obstacle_on_either_side_selects_opposite_clear_approach(self):
        for side in (-1, 1):
            g = self.grid()
            self.paint(g, lambda x, y: math.hypot(x-.65, y-.18*side) < .18)
            before = list(g.data)
            point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6, path_lookahead=0.)
            self.assertFalse(detour)
            self.assertLess(point[1]*side, -.1)
            self.assertTrue(g.clear(point[0], point[1], .12))
            self.assertEqual(before, g.data)

    def test_route_length_prefers_endpoint_near_shorter_way_around_wall(self):
        g = self.grid()
        # Straight-line scoring selects (0.9, 0); a wall blocks that approach.
        # Its lower end is 1.3 m away, whereas the upper end is only 0.2 m away.
        self.paint(g, lambda x, y: .42 <= x <= .48 and -1.3 <= y <= .2)
        from rgbd_following_core.motion import clear_standoff
        direct = clear_standoff(g, (0,0,0), (1.5,0), .6,.1,.5)
        self.assertAlmostEqual(direct[1], 0.)
        point, detour = reachable_standoff(g, (0,0,0), (1.5,0), .6,.5,.1,0., path_lookahead=0.)
        self.assertFalse(detour)
        self.assertGreater(point[1], .1)

    def test_soft_inflation_breaks_ties_without_becoming_a_hard_obstacle(self):
        g = self.grid()
        self.paint(g, lambda x, y: 0 < x < 1.2 and 0 < y < .8, 98)
        self.paint(g, lambda x, y: math.hypot(x-.65, y) < .18)
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0., path_lookahead=0.)
        self.assertFalse(detour)
        self.assertLess(point[1], -.2)
        # Soft-cost cells remain traversable when they are the only approach.
        self.paint(g, lambda x, y: .3 < x < 1.2 and -.8 < y < 0, 100)
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0., path_lookahead=0.)
        self.assertFalse(detour)
        self.assertGreater(point[1], .2)

    def test_u_shaped_dead_end_allows_route_that_initially_retreats(self):
        g = self.grid()
        # Robot faces the closed end. Exiting requires passing x < -0.6 first;
        # every forward/lateral direct route collides with the U's walls.
        self.paint(g, lambda x, y: (.22 <= x <= .28 and abs(y) <= .4) or
                   (-.6 <= x <= .28 and .35 <= abs(y) <= .4))
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0.,
                                          max_expansions=40000, path_lookahead=0.)
        self.assertFalse(detour)
        self.assertTrue(g.clear(point[0], point[1], .12))
        self.assertGreaterEqual(point[0], .6)
        # Closing the rear opening disconnects that same endpoint completely.
        self.paint(g, lambda x, y: -.65 <= x <= -.6 and abs(y) <= .4)
        self.assertIsNone(reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0.))

    def test_forward_waypoint_beats_normal_ring_requiring_initial_turn_away(self):
        g = self.grid()
        # Normal-distance endpoints beyond the U are connected through its rear
        # opening, but getting there requires first facing away from the leader.
        # A short forward intermediate endpoint is currently clear and useful.
        self.paint(g, lambda x, y: (.40 <= x <= .46 and abs(y) <= .4) or
                   (-.6 <= x <= .46 and .35 <= abs(y) <= .4))
        normal, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0.,
                                           max_expansions=40000, path_lookahead=0.)
        self.assertFalse(detour)
        self.assertGreaterEqual(normal[0], .6)
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6)
        self.assertTrue(detour)
        self.assertGreaterEqual(point[0], .1)
        self.assertLess(point[0], .3)
        self.assertLess(abs(point[1]), .05)
        self.assertTrue(g.clear(point[0], point[1], .12))
        self.assertGreater(math.hypot(point[0]-1.2,point[1]), .6)

    def test_multiple_obstacles_use_a_connected_offset_corridor(self):
        g = self.grid()
        self.paint(g, lambda x, y: (.35 <= x <= .40 and not -.22 < y < .22) or
                   (.80 <= x <= .85 and not .04 < y < .48))
        point, detour = reachable_standoff(g, (0,0,0), (1.8,0), .6,.5,.08,.6, path_lookahead=0.)
        self.assertFalse(detour)
        self.assertGreater(point[0], .9)
        self.assertTrue(g.clear(point[0], point[1], .08))

    def test_corridor_narrower_than_footprint_is_not_traversable(self):
        for half_width, passable in ((.10, False), (.18, True)):
            g = self.grid()
            self.paint(g, lambda x, y: .35 <= x <= .45 and abs(y) >= half_width)
            before = list(g.data)
            result = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0.)
            self.assertEqual(result is not None, passable)
            self.assertEqual(before, g.data)

    def test_bounded_search_fallback_preserves_connectivity_and_footprint(self):
        g = self.grid()
        self.paint(g, lambda x, y: .42 <= x <= .48 and -1.3 <= y <= .2)
        point, detour = reachable_standoff(g, (0,0,0), (1.5,0), .6,.5,.1,.6,
                                          max_expansions=100)
        self.assertTrue(g.clear(point[0], point[1], .1))
        self.assertTrue(detour)
        self.assertGreaterEqual(point[0], .05)
        self.assertLess(math.hypot(point[0],point[1]), .2)
        self.assertIsNone(reachable_standoff(g, (0,0,0), (1.5,0), .6,.5,.1,.6,
                                            max_expansions=1))
        self.paint(g, lambda x, y: .42 <= x <= .48)
        self.assertIsNone(reachable_standoff(g, (0,0,0), (1.5,0), .6,.5,.1,0.,
                                            max_expansions=1))

    def test_recorded_detour_can_continue_laterally_without_shrinking_gap(self):
        with open(os.path.join(os.path.dirname(__file__), 'fixtures/rgbd_cone_standoff.json')) as source:
            fixture = json.load(source)
        cfg = fixture['grid']
        cfg['data'] = [value for value,count in cfg.pop('runs') for unused in range(count)]
        g = Grid(**cfg)
        # This was the prior algorithm's first waypoint. Replanning from it
        # previously returned None because all smaller target rings were blocked.
        robot = (.23290733875902603, -.7376948772314267, -.6466477930134118)
        point, detour = reachable_standoff(g, robot, fixture['target'], .6,.5,.26,.6)
        self.assertTrue(detour)
        self.assertGreaterEqual(math.hypot(point[0]-robot[0], point[1]-robot[1]), .1)
        self.assertGreaterEqual(math.hypot(point[0]-fixture['target'][0],
                                          point[1]-fixture['target'][1]), .79)
        self.assertTrue(g.clear(point[0], point[1], .26))

    def test_long_selected_route_is_sent_as_a_nearby_waypoint(self):
        g = self.grid()
        for side in (-1, 1):
            g = self.grid()
            self.paint(g, lambda x, y: math.hypot(x-.65, y-.18*side) < .18)
            before = list(g.data)
            point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,.6)
            self.assertTrue(detour)
            self.assertLessEqual(math.hypot(point[0], point[1]), .36)
            self.assertGreaterEqual(math.hypot(point[0], point[1]), .20)
            self.assertTrue(g.clear(point[0], point[1], .12))
            self.assertGreaterEqual(math.hypot(point[0]-1.2, point[1]), .5)
            self.assertAlmostEqual(point[2], math.atan2(-point[1], 1.2-point[0]))
            self.assertEqual(before, g.data)

    def test_near_waypoint_uses_connected_path_instead_of_line_to_endpoint(self):
        g = self.grid()
        self.paint(g, lambda x, y: (.22 <= x <= .28 and abs(y) <= .4) or
                   (-.6 <= x <= .28 and .35 <= abs(y) <= .4))
        point, detour = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.12,0.,
                                          max_expansions=40000)
        self.assertTrue(detour)
        # The only exit is behind: interpolating toward the distant endpoint
        # would place the intermediate goal directly inside the U's front wall.
        self.assertLess(point[0], -.20)
        self.assertLess(abs(point[1]), .10)
        self.assertTrue(g.clear(point[0], point[1], .12))

    def test_near_waypoint_exclusions_cannot_be_bypassed_by_far_endpoint(self):
        g = self.grid()
        # Only the straight corridor is available; a previously visited/failed
        # point excludes the preferred lookahead interval, not the final ring.
        self.paint(g, lambda x, y: abs(y) > .15)
        result = reachable_standoff(g, (0,0,0), (1.2,0), .6,.5,.08,.6,
                                    excluded=[(.30,0,0)])
        if result is not None:
            point, detour = result
            self.assertTrue(detour)
            self.assertLessEqual(math.hypot(point[0], point[1]), .46)
            self.assertGreaterEqual(math.hypot(point[0]-.30, point[1]), .15)

    def test_short_final_approach_keeps_actual_standoff_endpoint(self):
        point, detour = reachable_standoff(self.grid(), (0,0,0), (.8,0), .6,.5,.12,.6)
        self.assertFalse(detour)
        self.assertAlmostEqual(point[0], .20)
        self.assertAlmostEqual(point[1], 0.)


if __name__ == '__main__':
    unittest.main()
