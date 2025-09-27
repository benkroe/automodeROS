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

    def create_config_files(self):
        self._create_config_file(
            self.behavior_list_client,
            "Behavior",
            "s",
            0,
            "behavior_categories_config.json"
        )
        self._create_config_file(
            self.condition_list_client,
            "Condition",
            "c",
            1,
            "condition_categories_config.json"
        )

    def _create_config_file(self, client, categories_name, categoryid, typeid, output_path):

        # Debug message
        if future.result() and future.result().success:
            raw_message = future.result().message
            self.get_logger().info(f"Raw service response: {raw_message}")
            try:
                descriptions = json.loads(raw_message)
            except Exception as e:
                self.get_logger().error(f"JSON decode error: {e}")
                return
            self.get_logger().info(f"Decoded descriptions: {descriptions}")
            config = self._format_config(descriptions, categories_name, categoryid, typeid)
            with open(output_path, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f"Config file written to {output_path}")






        if not client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f"{categories_name} list service not available!")
            return

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() and future.result().success:
            descriptions = json.loads(future.result().message)
            config = self._format_config(descriptions, categories_name, categoryid, typeid)
            with open(output_path, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f"Config file written to {output_path}")
        else:
            self.get_logger().error(f"Failed to get {categories_name.lower()} descriptions")

    def _format_config(self, descriptions, categories_name, categoryid, typeid):
        categories = []
        for idx, desc in enumerate(descriptions):
            if isinstance(desc, str):
                if not desc.strip():
                    continue
                try:
                    desc = json.loads(desc)
                except Exception as e:
                    self.get_logger().warn(f"Could not parse description: {desc} ({e})")
                    continue
            cat = {
                "name": desc.get("name", ""),
                "id": desc.get("id", idx),
                "display_name": desc.get("display_name", desc.get("name", "")),
                "params": desc.get("params", [])
            }
            categories.append(cat)
        return [{
            "typeid": typeid,
            "categoryid": categoryid,
            "categories_name": categories_name,
            "categories": categories
        }]

def main(args=None):
    rclpy.init(args=args)
    node = CategoriesCreator()
    node.create_config_files()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()