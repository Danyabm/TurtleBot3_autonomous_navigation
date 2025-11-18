#!/usr/bin/env python3
"""
Test trained RL agent in simulation
Watch your trained robot navigate autonomously!
"""

import os
import time
from stable_baselines3 import PPO
from rl_navigation_env import TurtleBotRLEnv
import numpy as np

def test_rl_agent(model_path=None, num_episodes=5):
    """
    Load trained model and test in environment
    """
    print("="*60)
    print("🤖 TESTING TRAINED RL AGENT")
    print("="*60)
    
    # Default model path
    if model_path is None:
        model_path = os.path.expanduser(
            "~/turtlebot3_ws/rl_models/ppo_turtlebot_final"
        )
    
    # Load trained model
    print(f"📂 Loading model from: {model_path}")
    try:
        model = PPO.load(model_path)
        print("✅ Model loaded successfully!")
    except:
        print("❌ Model not found! Train first with train_rl_agent.py")
        return
    
    # Create environment
    env = TurtleBotRLEnv(goal_position=(2.0, 2.0))
    
    # Test for multiple episodes
    success_count = 0
    collision_count = 0
    timeout_count = 0
    
    for episode in range(num_episodes):
        print(f"\n{'='*60}")
        print(f"📍 Episode {episode + 1}/{num_episodes}")
        print(f"{'='*60}")
        
        obs, info = env.reset()
        done = False
        episode_reward = 0
        steps = 0
        
        while not done:
            # Get action from trained policy (deterministic=True for testing)
            action, _ = model.predict(obs, deterministic=True)
            
            # Execute action
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1
            
            # Print progress every 50 steps
            if steps % 50 == 0:
                print(f"  Step {steps}: Distance to goal = {info['distance_to_goal']:.2f}m")
        
        # Episode finished
        print(f"\n📊 Episode Results:")
        print(f"  Total steps: {steps}")
        print(f"  Total reward: {episode_reward:.2f}")
        
        if info['goal_reached']:
            print(f"  Result: ✅ SUCCESS!")
            success_count += 1
        elif info['collision']:
            print(f"  Result: 💥 COLLISION")
            collision_count += 1
        else:
            print(f"  Result: ⏰ TIMEOUT")
            timeout_count += 1
    
    # Summary
    print(f"\n{'='*60}")
    print(f"📈 OVERALL PERFORMANCE ({num_episodes} episodes)")
    print(f"{'='*60}")
    print(f"✅ Successes:  {success_count} ({success_count/num_episodes*100:.1f}%)")
    print(f"💥 Collisions: {collision_count} ({collision_count/num_episodes*100:.1f}%)")
    print(f"⏰ Timeouts:   {timeout_count} ({timeout_count/num_episodes*100:.1f}%)")
    print(f"{'='*60}")
    
    env.close()

def demo_continuous():
    """
    Run agent continuously for live demo
    """
    model_path = os.path.expanduser("~/turtlebot3_ws/rl_models/ppo_turtlebot_final")
    
    print("🎮 CONTINUOUS DEMO MODE")
    print("Press Ctrl+C to stop\n")
    
    model = PPO.load(model_path)
    env = TurtleBotRLEnv(goal_position=(2.0, 2.0))
    
    try:
        while True:
            obs, info = env.reset()
            done = False
            
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, truncated, info = env.step(action)
            
            time.sleep(2)  # Pause between episodes
            
    except KeyboardInterrupt:
        print("\n👋 Demo stopped")
    finally:
        env.close()

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        demo_continuous()
    else:
        test_rl_agent(num_episodes=5)