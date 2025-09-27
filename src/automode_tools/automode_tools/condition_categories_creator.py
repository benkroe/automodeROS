#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json

class CategoriesCreator(Node):
    def __init__(self):
        super().__init__('categories_creator')
        self.behavior_list_client = self.create_client(Trigger, 'behaviors/list_srv')
        self.condition_list_client = self.create_client(Trigger, 'conditions/list_srv')

    def create_config_file(self, output_path='categories_config.json'):
        behaviors = self._get_descriptions(self.behavior_list_client, "Behavior")
        conditions = self._get_descriptions(self.condition_list_client, "Condition")
        config = []
        if behaviors:
            config.append(self._format_config(behaviors, "Behavior", "s", 0))
        if conditions:
            config.append(self._format_config(conditions, "Condition", "c", 1))
        with open(output_path, 'w') as f:
            json.dump(config, f, indent=2)
        self.get_logger().info(f"Config file written to {output_path}")

    def _get_descriptions(self, client, category_name):
        if not client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f"{category_name} list service not available!")
            return []
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() and future.result().success:
            return json.loads(future.result().message)
        else:
            self.get_logger().error(f"Failed to get {category_name.lower()} descriptions")
            return []

    def _format_config(self, descriptions, categories_name, categoryid, typeid):
        categories = []
        for idx, desc in enumerate(descriptions):
            cat = {
                "name": desc.get("name", ""),
                "id": desc.get("id", idx),
                "display_name": desc.get("display_name", desc.get("name", "")),
                "params": desc.get("params", [])
            }
            categories.append(cat)
        return {
            "typeid": typeid,
            "categoryid": categoryid,
            "categories_name": categories_name,
            "categories": categories
        }

def main(args=None):
    rclpy.init(args=args)
    node = CategoriesCreator()
    node.create_config_file()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()