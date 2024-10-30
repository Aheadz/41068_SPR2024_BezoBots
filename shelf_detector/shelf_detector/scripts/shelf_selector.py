#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk
import json
import threading

class ShelfSelectorGUI(Node):
    def __init__(self):
        super().__init__('shelf_selector')
        
        # Subscribe to shelf positions
        self.shelf_positions_sub = self.create_subscription(
            String,
            '/shelf_positions',
            self.shelf_positions_callback,
            10
        )
        
        # Publisher for selected shelf goal
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.shelf_positions = {}
        self.current_selection = None  # Track current selection
        
        # Create GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def shelf_positions_callback(self, msg):
        """Update shelf positions when new data is received"""
        try:
            # Store current selection if it exists
            current_selection = None
            if hasattr(self, 'shelf_listbox'):
                selections = self.shelf_listbox.curselection()
                if selections:
                    current_selection = self.shelf_listbox.get(selections[0])
            
            # Update shelf positions
            self.shelf_positions = json.loads(msg.data)
            
            # Update GUI shelf list if window exists
            if hasattr(self, 'shelf_listbox'):
                self.update_shelf_list(preserve_selection=current_selection)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding shelf positions: {str(e)}')

    def create_gui(self):
        """Create the GUI window"""
        self.window = tk.Tk()
        self.window.title("Shelf Selector")
        
        # Create main frame
        frame = ttk.Frame(self.window, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create listbox for shelves
        self.shelf_listbox = tk.Listbox(frame, height=10, width=30, exportselection=False)  # Added exportselection=False
        self.shelf_listbox.grid(row=0, column=0, padx=5, pady=5)
        
        # Create select button
        select_btn = ttk.Button(frame, text="Send to Robot", command=self.send_goal)
        select_btn.grid(row=1, column=0, pady=5)
        
        # Add status label
        self.status_label = ttk.Label(frame, text="Status: Waiting for selection...")
        self.status_label.grid(row=2, column=0, pady=5)
        
        # Add coordinates label
        self.coord_label = ttk.Label(frame, text="Selected: None")
        self.coord_label.grid(row=3, column=0, pady=5)
        
        # Add publisher status
        self.pub_status = ttk.Label(frame, text=f"Publishers on /goal_pose: {self.goal_pub.get_subscription_count()}")
        self.pub_status.grid(row=4, column=0, pady=5)
        
        # Bind selection event
        self.shelf_listbox.bind('<<ListboxSelect>>', self.on_select)
        
        # Create timer to update publisher status
        def update_pub_status():
            self.pub_status.config(text=f"Publishers on /goal_pose: {self.goal_pub.get_subscription_count()}")
            self.window.after(1000, update_pub_status)
        
        self.window.after(1000, update_pub_status)
        
        # Update shelf list
        self.update_shelf_list()
        
        # Start GUI main loop
        self.window.mainloop()

    def on_select(self, event):
        """Handle selection events"""
        if self.shelf_listbox.curselection():
            selection = self.shelf_listbox.get(self.shelf_listbox.curselection())
            self.current_selection = selection
            shelf_id = selection.split(':')[0].strip()
            if shelf_id in self.shelf_positions:
                data = self.shelf_positions[shelf_id]
                self.coord_label.config(text=f"Selected: ({data['x']:.2f}, {data['y']:.2f})")

    def update_shelf_list(self, preserve_selection=None):
        """Update the listbox with current shelf positions"""
        # Remember current selection index
        current_index = -1
        if preserve_selection:
            for i in range(self.shelf_listbox.size()):
                if self.shelf_listbox.get(i) == preserve_selection:
                    current_index = i
                    break
        
        # Update list
        self.shelf_listbox.delete(0, tk.END)
        for shelf_id, data in self.shelf_positions.items():
            self.shelf_listbox.insert(tk.END, 
                f"{shelf_id}: ({data['x']:.2f}, {data['y']:.2f})")
        
        # Restore selection if it existed
        if current_index >= 0 and current_index < self.shelf_listbox.size():
            self.shelf_listbox.select_set(current_index)
            self.shelf_listbox.see(current_index)  # Ensure the selection is visible

    def send_goal(self):
        """Send selected shelf position as goal"""
        selection = self.shelf_listbox.curselection()
        if not selection:
            self.status_label.config(text="Status: No shelf selected!")
            return
            
        # Get selected shelf ID
        shelf_id = list(self.shelf_positions.keys())[selection[0]]
        shelf_data = self.shelf_positions[shelf_id]
        
        # Create and publish goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = shelf_data['x']
        goal_pose.pose.position.y = shelf_data['y']
        goal_pose.pose.position.z = 0.0
        
        # Set orientation to default (facing forward)
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_pose)
        
        # Update status labels
        self.status_label.config(text=f"Status: Published goal for {shelf_id}")
        self.coord_label.config(text=f"Selected: ({shelf_data['x']:.2f}, {shelf_data['y']:.2f})")
        
        self.get_logger().info(f'Published goal for {shelf_id}: x={shelf_data["x"]:.2f}, y={shelf_data["y"]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ShelfSelectorGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()