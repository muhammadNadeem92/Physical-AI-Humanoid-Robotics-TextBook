# Module 5: The AI Robot Brain: NVIDIA Isaac Platform - Style Guide & Terminology

## Terminology Standards

### Core Isaac Platform Terms
- **Isaac Sim**: NVIDIA's simulation environment for robotics that provides photorealistic rendering and synthetic data generation capabilities. Always capitalize as "Isaac Sim" (not "IsaacSIM" or "Isaac-Sim").
- **Isaac ROS**: GPU-accelerated ROS packages that provide perception, navigation, and control capabilities using NVIDIA's hardware acceleration. Always capitalize as "Isaac ROS".
- **Omniverse**: NVIDIA's simulation and collaboration platform that enables USD-based scene description and realistic rendering. Always capitalize as "Omniverse".
- **USD (Universal Scene Description)**: When first mentioned in a document, use full form "Universal Scene Description (USD)". Subsequently, use "USD".
- **VSLAM (Visual SLAM)**: When first mentioned in a document, use full form "Visual SLAM (VSLAM)". Subsequently, use "VSLAM".

### Technical Terms
- **GPU acceleration**: Always use "GPU acceleration" (not "GPU-accelerated" when used as a noun phrase).
- **GPU-accelerated**: Use as an adjective (e.g., "GPU-accelerated perception").
- **Real-time** (not "real time"): Use hyphenated form when used as an adjective (e.g., "real-time processing").
- **Humanoid robot** (not "humanoid-robot"): Use space instead of hyphen.
- **Sim-to-real** (not "sim to real"): Use hyphens when used as an adjective or noun.

### Hardware Platforms
- **NVIDIA Jetson** (not "Jetson"): Always include "NVIDIA" when referring to Jetson platforms.
- **RTX** (not "RTX GPU"): When referring to NVIDIA's rendering technology, use "RTX" without "GPU" suffix.
- **CUDA**: Always capitalize as "CUDA" (Compute Unified Device Architecture).

## Writing Style

### Structure and Format
- Follow the standard chapter structure: **Introduction → Concepts → Examples → Summary**
- Use H2 headers (##) for main sections and H3 headers (###) for subsections
- Use bullet points for lists rather than numbered lists unless sequence is important
- Use bold formatting for key terms when first introduced in a section

### Technical Content
- Focus on systems, not math: Avoid deep neural network derivations and mathematical proofs
- Emphasize GPU acceleration benefits throughout content
- Compare workstation vs edge performance clearly
- Use diagrams heavily to explain concepts (prefer diagrams over text explanations)
- Each chapter must be standalone retrievable for RAG/chatbot systems

### Language and Tone
- Use beginner-friendly language while maintaining technical accuracy
- Explain technical concepts before introducing terminology
- Use active voice where possible
- Maintain consistent terminology throughout all chapters
- Avoid jargon without explanation

### Code Examples
- Include comments explaining key parts of code examples
- Use realistic variable names that match the context
- Include error handling where relevant for production use
- Use Python for ROS 2 examples and C# for Unity examples
- Format code blocks with appropriate language specification (python, csharp, yaml, etc.)

## Visual Elements

### Diagrams
- Include system architecture diagrams showing relationships between components
- Use consistent visual style across all diagrams
- Include labels and legends where necessary
- Place diagrams close to relevant text content

### Tables
- Use tables to compare different approaches or technologies
- Include clear headers and consistent formatting
- Use tables for side-by-side comparisons (e.g., Gazebo vs Isaac Sim)

## Cross-References

### Internal References
- Link to related concepts within Module 5 when relevant
- Reference Module 4 concepts when building upon previous material
- Maintain consistency with Module 2 (ROS 2) and Module 3 (simulation fundamentals) concepts

### External References
- Reference NVIDIA documentation for detailed technical specifications
- Link to official Isaac ROS packages and tools
- Include links to relevant research papers where appropriate

## Quality Standards

### Content Quality
- Each chapter must be understandable independently
- All claims must be accurate and verifiable
- Use headings, bullet points, and short paragraphs for readability
- Focus on testing, debugging, and validation rather than implementation details

### Technical Accuracy
- Verify all technical information against official NVIDIA documentation
- Ensure code examples follow best practices
- Include appropriate disclaimers for experimental or advanced features
- Test all practical examples before publication

## Special Considerations for Isaac Platform

### Performance Focus
- Emphasize performance benefits of GPU acceleration
- Include performance profiling examples
- Discuss resource constraints on edge platforms (Jetson)

### Perception Focus
- Highlight perception capabilities throughout content
- Emphasize synthetic data generation benefits
- Compare to traditional robotics approaches

### Learning-Based Control
- Explain reinforcement learning concepts at a systems level
- Avoid deep mathematical explanations
- Focus on practical applications and use cases

This style guide ensures consistency across all Module 5 content and maintains the high standards expected for the Physical AI Humanoid Robotics Textbook.