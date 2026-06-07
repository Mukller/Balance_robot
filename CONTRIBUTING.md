# Contributing to ESP32 Self-Balancing Robot

Thank you for your interest in contributing! This document provides guidelines and instructions for contributing.

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/YOUR_USERNAME/Balance_robot.git`
3. Create a branch: `git checkout -b feature/your-feature-name`
4. Make your changes
5. Commit: `git commit -m "feat: describe your changes"`
6. Push: `git push origin feature/your-feature-name`
7. Open a Pull Request

## Code Style

### Arduino/C++
- Use clear, descriptive variable names
- Include comments for non-obvious logic
- Follow the existing code structure
- Use 2-space indentation
- Keep functions focused and under 50 lines

### Naming Conventions
- Functions: `camelCase()` or `snake_case()`
- Variables: `snake_case`
- Constants: `UPPER_CASE`
- Classes: `PascalCase`

### Comments
```cpp
// Use single-line comments for brief explanations
// Multiple lines for complex logic

// Avoid over-commenting obvious code
// int x = 5;  // Set x to 5  ❌ Not needed

// Do comment the "why", not the "what"
// Calculate slope based on Z-axis to detect hills ✅
int16_t z_delta = current_z - baseline_z;
```

## Testing

Before submitting a pull request:

1. **Test on hardware** - Run your code on an actual ESP32
2. **Use test sketches** - Verify with test_*.ino files
3. **Check Serial output** - Verify debug messages
4. **Run all test sketches**:
   - test_line_sensor.ino
   - test_distance_sensor.ino
   - test_motors.ino

## Documentation

- Update README.md if you change user-facing features
- Update CHANGELOG.md with your changes
- Document new functions with brief comments
- Include GPIO pins in comments for hardware changes

## Commit Messages

Follow conventional commits format:

```
feat: Add new feature description
fix: Fix specific bug
docs: Update documentation
refactor: Refactor without changing behavior
test: Add or update tests
chore: Update dependencies, configs, etc.
```

Example:
```
feat: add web interface for robot control

- Add HTML/CSS/JS joystick interface
- Real-time sensor data display
- Support for mobile devices
- Update README with web interface docs
```

## Pull Request Process

1. **Title**: Brief description (max 70 chars)
2. **Description**: 
   - What changed and why
   - Testing done
   - Any breaking changes
   - Related issues

3. **Checklist**:
   - [ ] Code follows style guide
   - [ ] Changes are tested on hardware
   - [ ] Documentation is updated
   - [ ] No new warnings generated
   - [ ] Code is clean and readable

## Areas for Contribution

### Code
- [ ] Performance optimization
- [ ] Bug fixes
- [ ] New sensor support
- [ ] WiFi improvements
- [ ] Web interface enhancements

### Documentation
- [ ] Fix typos
- [ ] Improve clarity
- [ ] Add examples
- [ ] Translate to other languages
- [ ] Create video tutorials

### Testing
- [ ] Test edge cases
- [ ] Test on different ESP32 boards
- [ ] Test sensor combinations
- [ ] Performance benchmarks

### Examples
- [ ] Example sketches
- [ ] Advanced features
- [ ] Integration examples
- [ ] Troubleshooting guides

## Reporting Issues

When reporting bugs:

1. **Title**: Clear, descriptive
2. **Description**:
   - What happened
   - What should happen
   - Steps to reproduce
   - Hardware setup
   - Code version
3. **Logs**: Include Serial output
4. **Images**: Screenshots/photos if relevant

## Questions?

- Open an issue for questions
- Check existing issues first
- Include relevant code snippets
- Be specific about your setup

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Code of Conduct

Please follow our [Code of Conduct](CODE_OF_CONDUCT.md) in all interactions.

---

Thank you for making this project better! 🎉

**Maintainers:**
- Anton (@Mukller)

**Last Updated:** 2026-06-07
