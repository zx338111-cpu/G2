# Safety Rules for Humanoid Robot AI Workflow

AI-B / Codex must stop and ask the user before:

1. Running sudo.
2. Running rm -rf or destructive deletion.
3. Deleting datasets, checkpoints, rosbags, logs, source directories, or config files.
4. Reading .env, SSH keys, API keys, tokens, credentials, or private secrets.
5. Launching real robot motion.
6. Starting ROS hardware drivers, robot controllers, motor drivers, actuator commands, or manipulation scripts.
7. Changing torque, velocity, current, joint limits, safety filters, emergency stop, or low-level controller limits.
8. Installing system packages.
9. Pushing directly to main/master.
10. Running jobs over 2 hours.

Allowed without extra approval:

1. Reading normal project source files.
2. Creating docs under .ai_agents/.
3. Creating result JSON files under .ai_agents/tasks/results/.
4. Creating logs under .ai_agents/logs/.
5. Running static inspection commands.
6. Running unit tests or dry-runs when explicitly requested.
7. Analyzing existing logs.
8. Generating experiment plans and evaluation templates.

Real robot policy:

AI can prepare scripts, checklists, and analysis.
AI must not autonomously command real robot motion.
The user controls all real robot execution.
