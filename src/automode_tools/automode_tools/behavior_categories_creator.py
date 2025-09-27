#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json

class BehaviorCategoriesCreator(Node):
    def __init__(self):
        super().__init__('behavior_categories_creator')
        self.behavior_list_client = self.create_client(Trigger, 'behaviors/list_srv')

    def create_config_file(self, output_path='behavior_categories_config.json'):
        if not self.behavior_list_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Behavior list service not available!")
            return

        request = Trigger.Request()
        future = self.behavior_list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() and future.result().success:
            behavior_descriptions = json.loads(future.result().message)
            config = self._format_config(behavior_descriptions)
            with open(output_path, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f"Config file written to {output_path}")
        else:
            self.get_logger().error("Failed to get behavior descriptions")

    def _format_config(self, behavior_descriptions):
        categories = []
        for idx, desc in enumerate(behavior_descriptions):
            # If desc is a string, parse it as JSON
            if isinstance(desc, str):
                desc = json.loads(desc)
            cat = {
                "name": desc.get("name", ""),
                "id": desc.get("id", idx),
                "display_name": desc.get("display_name", desc.get("name", "")),
                "params": desc.get("params", [])
            }
            categories.append(cat)
        config = [{
            "typeid": 0,
            "categoryid": "s",
            "categories_name": "Behavior",
            "categories": categories
        }]
        return config

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCategoriesCreator()
    node.create_config_file()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()