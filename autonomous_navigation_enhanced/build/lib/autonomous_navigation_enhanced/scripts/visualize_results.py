#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# Example data - replace with your actual logged data
time = np.linspace(0, 60, 100)
efficiency = 85 + 10*np.sin(time/10) + np.random.randn(100)*2
speed = 0.2 + 0.05*np.sin(time/15) + np.random.randn(100)*0.02

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time, efficiency, 'b-', linewidth=2)
ax1.set_ylabel('Path Efficiency (%)', fontsize=12)
ax1.set_title('TurtleBot3 Enhanced Navigation Performance', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=80, color='r', linestyle='--', label='Target Efficiency')
ax1.legend()

ax2.plot(time, speed, 'g-', linewidth=2)
ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Speed (m/s)', fontsize=12)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('navigation_performance.png', dpi=300, bbox_inches='tight')
print("✅ Performance visualization saved!")

