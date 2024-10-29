#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import cv2
import yaml
import json
from scipy.spatial.distance import pdist, squareform

class CalibratedShelfDetector(Node):
    def __init__(self):
        super().__init__('calibrated_shelf_detector')
        
        # Known shelf dimensions based on measured distances
        self.SHELF_SIZE = 0.66  # meters
        self.POINT_DISTANCE_TOLERANCE = 0.1  # meters
        self.DEBUG = True
        
        # Gazebo coordinate adjustment
        self.GAZEBO_Y_OFFSET = -1.91  # Adjustment to match Gazebo coordinates
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_yaml_path', 'map.yaml'),
                ('binary_map_path', 'map_binary.pgm')
            ]
        )
        
        # Publishers
        self.position_pub = self.create_publisher(String, '/shelf_positions', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/shelf_markers', 10)
        self.debug_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)
        
        # Store detected positions
        self.shelf_positions = {}
        
        # Detect shelves
        self.detect_shelves()
        
        # Timer for publishing
        self.create_timer(1.0, self.publish_positions)

    def detect_shelves(self):
        """Detect shelves using known dimensions"""
        try:
            # Log the paths we're trying to open
            self.get_logger().info(f"Trying to open map YAML: {self.get_parameter('map_yaml_path').value}")
            self.get_logger().info(f"Trying to open binary map: {self.get_parameter('binary_map_path').value}")
            
            # Load map configuration
            with open(self.get_parameter('map_yaml_path').value, 'r') as f:
                map_data = yaml.safe_load(f)
            self.get_logger().info(f"YAML content: {map_data}")
            
            self.resolution = map_data['resolution']
            self.origin = map_data['origin']
            self.get_logger().info(f"Map resolution: {self.resolution}, origin: {self.origin}")
            
            # Load binary map
            binary_map = cv2.imread(
                self.get_parameter('binary_map_path').value,
                cv2.IMREAD_GRAYSCALE
            )
            
            if binary_map is None:
                self.get_logger().error("Failed to load binary map")
                return
                
            self.get_logger().info(f"Binary map shape: {binary_map.shape}")
            self.get_logger().info(f"Binary map unique values: {np.unique(binary_map)}")
            
            # Find all points (small circles in the map)
            _, binary = cv2.threshold(binary_map, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(
                binary, 
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            self.get_logger().info(f"Found {len(contours)} contours")
            
            # Get centroids of all small circles
            points = []
            for contour in contours:
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # Convert to map coordinates with Gazebo adjustment
                    map_x = cx * self.resolution + self.origin[0]
                    map_y = (cy * self.resolution + self.origin[1]) + self.GAZEBO_Y_OFFSET
                    points.append((map_x, map_y))
            
            self.get_logger().info(f"Found {len(points)} potential shelf points")
            
            # Convert to numpy array
            points = np.array(points)
            
            # Calculate distances between all points
            distances = squareform(pdist(points))
            
            # Debug: Print some distance information
            if self.DEBUG and len(points) > 0:
                # Print some sample distances
                self.get_logger().info(f"Sample distances between points:")
                for i in range(min(5, len(points))):
                    for j in range(i+1, min(6, len(points))):
                        dist = distances[i][j]
                        self.get_logger().info(f"Distance between point {i} and {j}: {dist:.3f} meters")
                
                # Print nearby points for first few points
                for i in range(min(5, len(points))):
                    nearby = np.where(
                        (distances[i] > self.SHELF_SIZE - self.POINT_DISTANCE_TOLERANCE) &
                        (distances[i] < self.SHELF_SIZE + self.POINT_DISTANCE_TOLERANCE)
                    )[0]
                    self.get_logger().info(f"Point {i} has {len(nearby)} nearby points at the right distance")
            
            # Find groups of 4 points that form squares
            shelf_count = 0
            used_points = set()
            
            # Store all detected centers for duplicate filtering
            detected_centers = []
            
            for i in range(len(points)):
                if i in used_points:
                    continue
                    
                # Find points that are approximately SHELF_SIZE distance away
                nearby_points = np.where(
                    (distances[i] > self.SHELF_SIZE - self.POINT_DISTANCE_TOLERANCE) &
                    (distances[i] < self.SHELF_SIZE + self.POINT_DISTANCE_TOLERANCE)
                )[0]
                
                if len(nearby_points) >= 3:  # Need at least 3 other points
                    # Verify it forms a shelf pattern
                    potential_square = [i] + list(nearby_points[:3])
                    if self.is_valid_shelf(points[potential_square]):
                        # Calculate center
                        center = np.mean(points[potential_square], axis=0)
                        
                        # Check if this center is too close to any existing centers
                        is_duplicate = False
                        for existing_center in detected_centers:
                            if np.linalg.norm(center - existing_center) < self.SHELF_SIZE:
                                is_duplicate = True
                                break
                        
                        if not is_duplicate:
                            detected_centers.append(center)
                            self.shelf_positions[f'shelf_{shelf_count}'] = {
                                'x': float(center[0]),
                                'y': float(center[1]),
                                'corner_points': points[potential_square].tolist()
                            }
                            
                            used_points.update(potential_square)
                            shelf_count += 1
            
            # Sort shelves by y-coordinate to separate top and bottom sections
            if self.shelf_positions:
                centers = np.array([data['y'] for data in self.shelf_positions.values()])
                threshold = (np.max(centers) + np.min(centers)) / 2
                
                top_shelves = {k: v for k, v in self.shelf_positions.items() 
                              if v['y'] > threshold}
                bottom_shelves = {k: v for k, v in self.shelf_positions.items() 
                                if v['y'] <= threshold}
                
                self.get_logger().info(f'Found {len(top_shelves)} shelves in top section')
                self.get_logger().info(f'Found {len(bottom_shelves)} shelves in bottom section')
            
            self.get_logger().info(f'Detected {shelf_count} shelves total')
            
            # Save results
            with open('detected_shelf_positions.json', 'w') as f:
                json.dump(self.shelf_positions, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f'Error detecting shelves: {str(e)}')

    def is_valid_shelf(self, points):
        """Verify if 4 points form a valid shelf pattern"""
        if len(points) != 4:
            return False
            
        # Calculate all pairwise distances
        distances = sorted(pdist(points))
        
        # We expect:
        # - At least 2 distances around 0.66-0.67m (sides)
        # - Remaining distances should be larger (diagonals)
        small_distances = [d for d in distances if abs(d - self.SHELF_SIZE) < self.POINT_DISTANCE_TOLERANCE]
        
        if self.DEBUG:
            self.get_logger().info(f"Checking points with distances: {[f'{d:.3f}' for d in distances]}")
            self.get_logger().info(f"Found {len(small_distances)} distances matching shelf size")
        
        # We should have at least 2 sides matching our shelf size
        return len(small_distances) >= 2

    def publish_positions(self):
        """Publish shelf positions and visualization markers"""
        if not self.shelf_positions:
            return
            
        # Define the valid middle section region to match Gazebo coordinates
        MIDDLE_X_MIN = -3.0
        MIDDLE_X_MAX = 1.0
        
        # Filter shelf positions to only include those in the middle section
        valid_shelf_positions = {
            shelf_id: data for shelf_id, data in self.shelf_positions.items()
            if MIDDLE_X_MIN <= data['x'] <= MIDDLE_X_MAX
        }
        
        # Add debug logging with Gazebo coordinates
        for shelf_id, data in self.shelf_positions.items():
            self.get_logger().info(f"Shelf {shelf_id} at x={data['x']:.3f}, y={data['y']:.3f} - " + 
                                 ("Valid" if MIDDLE_X_MIN <= data['x'] <= MIDDLE_X_MAX else "Filtered out"))
        
        self.get_logger().info(f"Filtered from {len(self.shelf_positions)} to {len(valid_shelf_positions)} shelves")
        
        # Publish positions
        pos_msg = String()
        pos_msg.data = json.dumps(valid_shelf_positions)
        self.position_pub.publish(pos_msg)
        
        # Create visualization markers
        marker_array = MarkerArray()
        debug_array = MarkerArray()
        
        # Only visualize valid shelves
        for shelf_id, data in valid_shelf_positions.items():
            # Shelf center marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "shelves"
            marker.id = int(shelf_id.split('_')[1])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = data['x']
            marker.pose.position.y = data['y']
            marker.pose.position.z = 0.5
            
            marker.scale.x = self.SHELF_SIZE
            marker.scale.y = self.SHELF_SIZE
            marker.scale.z = 1.0
            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            marker_array.markers.append(marker)
            
            # Debug markers for corner points
            for i, point in enumerate(data['corner_points']):
                debug_marker = Marker()
                debug_marker.header.frame_id = "map"
                debug_marker.header.stamp = self.get_clock().now().to_msg()
                debug_marker.ns = f"shelf_{shelf_id}_corners"
                debug_marker.id = i
                debug_marker.type = Marker.SPHERE
                debug_marker.action = Marker.ADD
                
                debug_marker.pose.position.x = point[0]
                debug_marker.pose.position.y = point[1]
                debug_marker.pose.position.z = 0.1
                
                debug_marker.scale.x = 0.1
                debug_marker.scale.y = 0.1
                debug_marker.scale.z = 0.1
                
                debug_marker.color.r = 1.0
                debug_marker.color.g = 0.0
                debug_marker.color.b = 0.0
                debug_marker.color.a = 1.0
                
                debug_array.markers.append(debug_marker)
        
        self.marker_pub.publish(marker_array)
        self.debug_pub.publish(debug_array)

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedShelfDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()