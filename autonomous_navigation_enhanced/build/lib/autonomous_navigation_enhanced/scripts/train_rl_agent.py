#!/usr/bin/env python3
"""
Train Deep RL agent for TurtleBot3 navigation
Uses PPO (Proximal Policy Optimization) - state-of-the-art RL algorithm
"""

import os
import time
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import torch

# Import our custom environment
from rl_navigation_env import TurtleBotRLEnv

def make_env():
    """Create and wrap environment"""
    env = TurtleBotRLEnv(goal_position=(2.0, 2.0))
    env = Monitor(env)
    return env

def train_rl_agent():
    """
    Train RL agent using PPO algorithm
    """
    print("="*60)
    print("🧠 DEEP REINFORCEMENT LEARNING TRAINING")
    print("="*60)
    print("Algorithm: Proximal Policy Optimization (PPO)")
    print("Neural Network: 2 hidden layers [256, 256]")
    print("Observation: 24 laser readings + 2 goal coordinates")
    print("Action: Continuous [linear_vel, angular_vel]")
    print("="*60)
    
    # Create directories
    models_dir = os.path.expanduser("~/turtlebot3_ws/rl_models")
    logs_dir = os.path.expanduser("~/turtlebot3_ws/rl_logs")
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    
    # Create environment
    env = DummyVecEnv([make_env])
    
    # Define PPO model with neural network
    model = PPO(
        "MlpPolicy",  # Multi-Layer Perceptron policy
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,  # Discount factor
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,  # Entropy coefficient (exploration)
        vf_coef=0.5,  # Value function coefficient
        max_grad_norm=0.5,
        policy_kwargs=dict(
            net_arch=[256, 256],  # Neural network architecture
            activation_fn=torch.nn.ReLU
        ),
        verbose=1,
        tensorboard_log=logs_dir
    )
    
    print("\n📊 Model Architecture:")
    print(model.policy)
    
    # Setup callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=models_dir,
        name_prefix="ppo_turtlebot"
    )
    
    # Training parameters
    total_timesteps = 100000  # Adjust based on your patience
    
    print(f"\n🚀 Starting training for {total_timesteps} timesteps...")
    print("💡 TIP: Open another terminal and run:")
    print(f"    tensorboard --logdir {logs_dir}")
    print("    Then visit http://localhost:6006 to watch training live!")
    print("\n⏰ This will take 1-2 hours. Go grab coffee! ☕\n")
    
    start_time = time.time()
    
    try:
        # TRAIN THE AGENT!
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=True
        )
        
        training_time = time.time() - start_time
        
        # Save final model
        final_model_path = os.path.join(models_dir, "ppo_turtlebot_final")
        model.save(final_model_path)
        
        print("\n" + "="*60)
        print("✅ TRAINING COMPLETE!")
        print(f"⏱️  Training time: {training_time/60:.1f} minutes")
        print(f"💾 Model saved to: {final_model_path}")
        print("="*60)
        
        # Print stats
        print("\n📈 Training Statistics:")
        print(f"Total timesteps: {total_timesteps}")
        print(f"Episodes completed: ~{total_timesteps/500}")  # Approx
        
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
        model.save(os.path.join(models_dir, "ppo_turtlebot_interrupted"))
        print("💾 Model saved (interrupted)")
    
    finally:
        env.close()

if __name__ == '__main__':
    train_rl_agent()