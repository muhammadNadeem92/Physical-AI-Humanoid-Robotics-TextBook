# Module 6: Vision–Language–Action (VLA) Systems - Style Guide & Terminology

## Terminology Standards

### Core VLA Terms
- **VLA (Vision-Language-Action)**: When first mentioned in a document, use full form "Vision-Language-Action (VLA)". Subsequently, use "VLA".
- **Vision-Language-Action (VLA) Systems**: The integrated systems that connect visual perception, language understanding, and physical action.
- **Embodied Cognition**: Always use full term "embodied cognition" when referring to the concept that intelligence emerges from interaction with the physical environment.
- **Perception-Planning-Action Loop**: Use full term when referring to the fundamental cycle of VLA systems.

### Technical Terms
- **LLM Planner**: Use this term to refer to large language models used specifically for task planning, not direct motor control.
- **ROS 2 Actions**: Always capitalize as "ROS 2 Actions" when referring to the ROS 2 action interface.
- **Symbol Grounding**: Use full term "symbol grounding" when referring to connecting abstract symbols to physical entities.
- **Safety-First**: Use hyphenated form when used as an adjective (e.g., "safety-first approach").

### System Architecture Terms
- **Planner vs Controller**: Always distinguish between "planner" (high-level task decomposition) and "controller" (low-level motor execution).
- **Task Decomposition**: Use this term for breaking high-level commands into detailed execution steps.
- **Constraint Enforcement**: Use this term for ensuring plans meet safety and feasibility requirements.

## Writing Style

### Structure and Format
- Follow the standard chapter structure: **Introduction → Concepts → Examples → Summary**
- Use H2 headers (##) for main sections and H3 headers (###) for subsections
- Use bullet points for lists rather than numbered lists unless sequence is important
- Use bold formatting for key terms when first introduced in a section

### Technical Content
- Emphasize that **LLMs are planners, not motor controllers**
- Focus on system design over implementation details
- Emphasize determinism, validation, and safety throughout content
- Avoid "prompt magic" - focus on architectural patterns
- Each chapter must be **semantically chunked** for RAG/retrieval

### Language and Tone
- Use beginner-friendly language while maintaining technical accuracy
- Explain technical concepts before introducing terminology
- Use active voice where possible
- Maintain consistent terminology throughout all chapters
- Avoid jargon without explanation

### Code Examples
- Include comments explaining key parts of code examples
- Use realistic variable names that match the context
- Include error handling and safety checks where relevant
- Use Python for ROS 2 examples and appropriate formats for LLM interactions
- Format code blocks with appropriate language specification (python, json, yaml, etc.)

## Visual Elements

### Diagrams
- Include system architecture diagrams showing the VLA integration
- Use consistent visual style across all diagrams
- Include labels and legends where necessary
- Place diagrams close to relevant text content
- Show the Voice → Language → Plan → ROS Actions → Robot flow

### Tables
- Use tables to compare different approaches or technologies
- Include clear headers and consistent formatting
- Use tables for side-by-side comparisons (e.g., chatbot vs embodied agent)

## Cross-References

### Internal References
- Link to related concepts within Module 6 when relevant
- Reference Module 5 concepts when building upon Isaac Platform knowledge
- Maintain consistency with Module 2 (ROS 2) concepts

### External References
- Reference LLM best practices for planning systems
- Link to ROS 2 action interface documentation
- Include links to speech-to-text service documentation where appropriate

## Quality Standards

### Content Quality
- Each chapter must be understandable independently
- All claims must be accurate and verifiable
- Use headings, bullet points, and short paragraphs for readability
- Focus on safety, determinism, and system design rather than implementation details

### Technical Accuracy
- Verify all technical information against best practices
- Ensure examples emphasize the planner vs controller distinction
- Include appropriate disclaimers for safety-critical aspects
- Test all practical examples before publication

## Special Considerations for VLA Systems

### Safety Focus
- Emphasize safety boundaries throughout content
- Highlight the importance of plan validation
- Include failure recovery strategies in all examples

### Planning vs Execution
- Maintain clear distinction between planning and execution layers
- Focus on high-level task decomposition rather than low-level control
- Emphasize the importance of constraint enforcement

### Perception Integration
- Highlight how language is grounded in visual perception
- Explain the symbol grounding problem and solutions
- Show how plans are validated against real-world perception

This style guide ensures consistency across all Module 6 content and maintains the high standards expected for the Physical AI Humanoid Robotics Textbook.