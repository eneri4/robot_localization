import time
import math
import pytest
import numpy as np
from unittest.mock import MagicMock, patch
import rclpy

from pf import Particle, ParticleFilter

@pytest.fixture
def mock_pf():
    """Fixture to instantiate a ParticleFilter with mocked ROS2 and helper components. 
    Avoids real ROS2 node creation, publishers, subscriptions, and threading during tests.
    *used gemini for help"""
    if not rclpy.ok():
        rclpy.init()
        
    # Apply all patches BEFORE instantiating ParticleFilter so it doesn't hang or hit real hardware/services
    with patch('pf.OccupancyField', autospec=True), \
         patch('pf.TFHelper', autospec=True), \
         patch('threading.Thread', autospec=True), \
         patch('rclpy.node.Node.create_publisher'), \
         patch('rclpy.node.Node.create_subscription'), \
         patch('rclpy.node.Node.create_timer'), \
         patch('rclpy.client.Client.wait_for_service', return_value=True):
        
        node = ParticleFilter()
        
        # Initialize default test attributes
        node.n_particles = 50
        node.d_thresh = 0.2
        node.a_thresh = math.pi / 6
        node.sigma = 0.4
        node.particle_cloud = []
        node.current_odom_xy_theta = []
        
        yield node
        
        # Teardown
        try:
            node.destroy_node()
        except Exception:
            pass


def test_init_particle_count(mock_pf):
    """Ensure particle cloud length matches n_particles."""
    mock_pf.initialize_particle_cloud(timestamp=None, xy_theta=(0.0, 0.0, 0.0))
    assert len(mock_pf.particle_cloud) == mock_pf.n_particles

def test_init_weights_normalized(mock_pf):
    """Ensure particle weights are normalized upon initialization."""
    mock_pf.initialize_particle_cloud(timestamp=None, xy_theta=(0.0, 0.0, 0.0))
    total_weight = sum(p.w for p in mock_pf.particle_cloud)
    assert math.isclose(total_weight, 1.0, rel_tol=1e-5)

def test_normalize_sums_to_one(mock_pf):
    """Ensure particle weights sum to 1 after normalization."""
    mock_pf.particle_cloud = [Particle(w=2.0), Particle(w=3.0), Particle(w=5.0)]
    mock_pf.normalize_particles()
    total = sum(p.w for p in mock_pf.particle_cloud)
    assert math.isclose(total, 1.0, rel_tol=1e-5)

def test_moved_below_threshold(mock_pf):
    """Returns False when movement is smaller than distance/angle thresholds."""
    mock_pf.current_odom_xy_theta = [0.0, 0.0, 0.0]
    assert mock_pf.moved_far_enough_to_update([0.1, 0.1, 0.1]) is False

def test_moved_exceeded_threshold(mock_pf):
    """Returns True when linear movement exceeds d_thresh."""
    mock_pf.current_odom_xy_theta = [0.0, 0.0, 0.0]
    assert mock_pf.moved_far_enough_to_update([0.3, 0.0, 0.0]) is True

def test_laser_skip_invalid_rays(mock_pf):
    """Ensures inf or nan scan rays are ignored without crashing."""
    mock_pf.particle_cloud = [Particle(x=0.0, y=0.0, theta=0.0)]
    mock_pf.occupancy_field.get_closest_obstacle_distance.return_value = 1.0
    
    r = [float('inf'), float('nan'), 2.0] + [1.0] * 357
    theta = [0.0] * 360
    
    try:
        mock_pf.update_particles_with_laser(r, theta)
    except Exception as e:
        pytest.fail(f"Laser update failed on invalid rays: {e}")

def test_pose_weighted_mean(mock_pf):
    """Ensures weighted mean position calculation is accurate."""
    mock_pf.particle_cloud = [
        Particle(x=0.0, y=0.0, theta=0.0, w=0.5),
        Particle(x=2.0, y=2.0, theta=0.0, w=0.5)
    ]
    mock_pf.update_robot_pose()
    assert math.isclose(mock_pf.robot_pose.position.x, 1.0, abs_tol=1e-5)
    assert math.isclose(mock_pf.robot_pose.position.y, 1.0, abs_tol=1e-5)

def test_laser_update_efficiency(mock_pf):
    """Asserts that updating 300 particles with a full laser scan 
       completes within an acceptable time limit (e.g., under 300 milliseconds).
    """
    mock_pf.n_particles = 300
    mock_pf.particle_cloud = [Particle(x=0.0, y=0.0, theta=0.0) for _ in range(mock_pf.n_particles)]
    mock_pf.occupancy_field.get_closest_obstacle_distance.return_value = 1.0
    
    r = [1.0] * 360
    theta = [0.0] * 360

    start_time = time.perf_counter()
    
    mock_pf.update_particles_with_laser(r, theta)
    
    elapsed_time = time.perf_counter() - start_time
    
    print(f"\n[Performance] Laser update for {mock_pf.n_particles} particles took: {elapsed_time * 1000:.2f} ms")

    assert elapsed_time < 0.3, f"Performance warning: Laser update took too long ({elapsed_time:.4f}s)"