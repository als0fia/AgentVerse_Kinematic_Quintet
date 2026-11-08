from __future__ import annotations

import tkinter as tk
import unittest

from ricebot.viewer.live_app import LiveSwarmDashboardApp


class TestLiveGuiMainMenuAndScalableFleet(unittest.TestCase):
    def test_main_menu_fleet_size_map_scale_and_belief_selector(self) -> None:
        root = tk.Tk()
        root.withdraw()
        try:
            app = LiveSwarmDashboardApp(
                root,
                seed=7,
                scenario="nominal",
                policy="mycelial",
                num_rovers=4,
                map_scale=1.0,
            )
            self.assertEqual(app.num_rovers, 4)
            self.assertEqual(len(app.frame.robots), 4)
            active_rids = [r.robot_id for r in app.frame.robots]
            self.assertEqual(active_rids, ["planter", "scout", "planter_2", "scout_2"])

            # Verify belief selector buttons exist for fused + all 4 rovers + divergence + ground_truth
            for expected_key in ("fused", "planter", "scout", "planter_2", "scout_2", "divergence", "ground_truth"):
                self.assertIn(expected_key, app.belief_radio_buttons)

            # Step a few times and verify per-rover and fused belief inspection + 2D rendering
            for _ in range(4):
                app.step_once()

            for view_key in ("fused", "planter", "scout", "planter_2", "scout_2", "divergence", "ground_truth"):
                app.select_belief_robot(view_key)
                self.assertEqual(app.selected_belief_robot, view_key)
                if view_key in active_rids:
                    self.assertEqual(app.selected_robot_id, view_key)

            # Verify elevation_m layer renders per-rover sensed confidence vs ground_truth
            app.layer_var.set("elevation_m")
            app.select_belief_robot("planter_2")
            app._render_2d_belief_map()

            # Switch fleet size and map scale dynamically from Main Menu
            app.apply_fleet_and_map_setup(num_rovers=3, map_scale=1.6)
            self.assertEqual(app.num_rovers, 3)
            self.assertAlmostEqual(app.map_scale, 1.6, places=4)
            self.assertEqual([r.robot_id for r in app.frame.robots], ["planter", "scout", "planter_2"])
            self.assertIn("planter_2", app.belief_radio_buttons)
            self.assertNotIn("scout_2", app.belief_radio_buttons)

            app.step_once()
            app.set_fleet_size(2)
            self.assertEqual(len(app.frame.robots), 2)
        finally:
            try:
                if hasattr(app, "renderer") and app.renderer is not None:
                    app.renderer.close()
            except Exception:
                pass
            root.destroy()


if __name__ == "__main__":
    unittest.main()
