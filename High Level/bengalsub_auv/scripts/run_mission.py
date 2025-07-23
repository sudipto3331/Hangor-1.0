"""
Script to run AUV mission from command line
Usage: python scripts/run_mission.py
"""

import sys
import os
import asyncio

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import main

if __name__ == "__main__":
    print("ROBOSUB25 AUV Mission Launcher")
    print("==============================")
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    except Exception as e:
        print(f"Mission failed: {e}")