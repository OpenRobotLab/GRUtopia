# Agents

> This is a directory for Agents.

# Import New Agent

When adding an Agent, you need to follow these steps:

1. Create a subclass of `grutopia.core.agent.BaseAgent` like `grutopia_extension.agents.dummy_agent`.
2. Implement `decision_making` method.
3. If necessary, you can create directories to implement the complex logic you need.

> We will officially integrate some Agents into this project as submodules (like `npc_agent`)
