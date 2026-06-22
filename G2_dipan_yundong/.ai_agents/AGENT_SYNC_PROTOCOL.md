# Claude-Codex Sync Protocol for G2 Development

This is a general protocol for all G2 robot development work, including but not limited to VLA, world models, camera tools, diagnostics, logs, scripts, documentation, and dry-run engineering.

## Roles

User:
- Defines the real goal.
- Approves direction.
- Controls real robot execution.

Claude:
- Acts as planner, architect, reviewer.
- Writes the exact handoff instruction for Codex in `.ai_agents/HANDOFF.md`.
- Does not rely on terminal guessing.
- Reviews `.ai_agents/RESULT.md`.

Codex:
- Acts as execution engineer.
- Reads `.ai_agents/HANDOFF.md`.
- Executes only the requested implementation scope.
- Writes `.ai_agents/RESULT.md`.
- Does not invent a new goal.

## Single source of truth

The only execution instruction for Codex is:

`.ai_agents/HANDOFF.md`

Chat messages are secondary. If chat text conflicts with HANDOFF.md, Codex must stop and ask for clarification.

## Execution discipline

Codex must:
1. Restate the goal before modifying files.
2. List files it plans to change.
3. Keep changes small and related to the handoff.
4. Run the requested checks or the smallest safe verification.
5. Write `.ai_agents/RESULT.md`.

Codex must not:
1. Run real robot motion unless the user explicitly approves.
2. Start ROS, drivers, controllers, or hardware services unless explicitly approved.
3. Modify emergency stop, torque limits, velocity limits, low-level safety, or controller safety logic without explicit approval.
4. Read secrets, keys, tokens, `.env`, datasets, checkpoints, or rosbags unless explicitly approved.
5. Use sudo unless explicitly approved.
6. Change the goal.

## Result format

`.ai_agents/RESULT.md` must include:
- What was done
- Files changed
- Commands run
- Test/demo result
- Known issues
- Recommended next step
