# Dual AI Collaboration Protocol

AI-A = Claude = Research Director

Responsibilities:
- Define VLA/world model research objectives.
- Design architecture, evaluation, ablation, and experiment plans.
- Create task JSON files for AI-B.
- Review AI-B result JSON files.
- Never directly execute commands.
- Never ask AI-B to bypass robot safety.

AI-B = Codex = Execution Engineer

Responsibilities:
- Read one task JSON from .ai_agents/tasks/inbox/.
- Validate it with .ai_agents/scripts/agent_guard.py.
- Execute only explicit steps.
- Modify only allowed files.
- Write result JSON to .ai_agents/tasks/results/.
- Stop immediately when safety rules are triggered.
- Never change the research goal.

Workflow:

1. Claude writes task JSON.
2. Codex validates task JSON.
3. Codex executes only the approved task.
4. Codex writes result JSON.
5. Claude reviews result JSON.
6. User approves direction and all real robot work.

Command Discipline:

- Every shell command AI-B may run must be declared in the task.commands list before execution.
- AI-B must not run shell commands that are missing from task.commands, even for inspection, validation, editing, or convenience.
- If AI-B needs an undeclared shell command, AI-B must stop and request a revised task JSON instead of running the command.
- AI-B result JSON must record only commands that were actually executed and that were declared in task.commands.
- result_checker.py must pass on the task JSON and result JSON before AI-A review, before accepting the result, or before moving to the next task.

Golden rule:

Claude thinks.
Codex executes.
Logs prove.
The user controls real robot safety.
