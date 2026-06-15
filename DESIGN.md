# Design

## Source of truth
- Status: Draft
- Last refreshed: 2026-06-13
- Primary product surfaces: Azas voice order screen, menu preview panel, robot pipeline status UI, kiosk/menu surfaces.
- Evidence reviewed: `src/azas_voice/web/voice.html`, `src/azas_voice/web/voice.css`, `src/azas_voice/web/voice.js`, `src/azas_voice/azas_voice/voice_screen_node.py`, `src/azas_voice/azas_voice/voice_pipeline_executor_node.py`, `src/azas_voice/config/recipes.yaml`, `src/azas_kiosk/`.

## Brand
- Personality: calm, precise, service-oriented cocktail robot.
- Trust signals: visible order confirmation, clear recipe ingredients, robot process status, failure/resume state visibility.
- Avoid: marketing hero pages, decorative-only UI, hidden robot motion state, fake coordinates or unsupported safety claims.

## Product goals
- Goals: let users order many named drinks by voice or touch, preview the finished drink, and understand the robot's current manufacturing stage.
- Non-goals: manual robot coordinate entry, free-form motion generation, unsupported recipe execution outside measured dispenser/color mappings.
- Success signals: users can pick from a larger menu, see ingredient amounts, see the current robot step, and recover from interrupted dispenser sequences.

## Personas and jobs
- Primary personas: demo operator, guest ordering a drink, developer validating the robot flow.
- User jobs: choose or request a drink, confirm execution, monitor robot progress, understand a stopped/resumed run.
- Key contexts of use: local robot station, ROS launch-driven demo, touchscreen or browser view near the robot.

## Information architecture
- Primary navigation: single voice order screen with adjacent menu/status panel.
- Core routes/screens: voice conversation, selected drink preview, robot process stage, catalog list.
- Content hierarchy: current order and confirmation first, finished drink preview second, recipe catalog and process detail nearby.

## Design principles
- Principle 1: show operational state directly instead of explaining the system.
- Principle 2: make the drink choice visual and scannable without hiding execution readiness.
- Tradeoffs: favor compact, reliable status over decorative immersion; prefer symbolic recipe/color data over robot-coordinate exposure.

## Visual language
- Color: ingredient colors map consistently to red/juice, yellow/syrup, green/liqueur, blue/rum.
- Typography: readable dashboard sizing; compact headings inside panels.
- Spacing/layout rhythm: two-column desktop layout with stacked mobile flow.
- Shape/radius/elevation: restrained panels and item cards; avoid nested card-on-card layouts.
- Motion: small process animations for robot stage changes; keep motion nonessential and readable.
- Imagery/iconography: HTML/SVG drink preview and CSS robot scene are acceptable when real finished-drink photos are unavailable.

## Components
- Existing components to reuse: voice orb, dialogue bubbles, status grid, recipe glass SVG, ingredient chips, pipeline step list.
- New/changed components: catalog item buttons, drink stat block, robot process scene, resume-aware pipeline stage.
- Variants and states: idle, recommended, confirmed, making, completed, failed, dry-run, resume recovery.
- Token/component ownership: `src/azas_voice/web/voice.css` owns current web styling; recipe data comes from `src/azas_voice/config/recipes.yaml`.

## Accessibility
- Target standard: practical WCAG AA for text contrast and keyboard/touch operation where possible.
- Keyboard/focus behavior: catalog entries and test form controls must remain button/input elements with visible focus.
- Contrast/readability: status text and badges must remain readable over panel backgrounds.
- Screen-reader semantics: use section labels and meaningful button labels for menu order actions.
- Reduced motion and sensory considerations: animations should be decorative and not required for understanding status.

## Responsive behavior
- Supported breakpoints/devices: desktop browser near robot, tablet/touch display, narrow mobile fallback.
- Layout adaptations: voice and menu panels stack on narrow screens; catalog remains scrollable.
- Touch/hover differences: catalog buttons must be usable without hover-only affordances.

## Interaction states
- Loading: retain previous state until fresh `/api/state` arrives.
- Empty: show no selected recipe and invite a voice/test utterance.
- Error: show pipeline/status failure and last known stage when available.
- Success: show completed badge and final drink preview.
- Disabled: hardware execution may remain dry-run from launch parameters.
- Offline/slow network, if applicable: periodic refresh should keep the last known UI state visible.

## Content voice
- Tone: concise Korean service copy.
- Terminology: use menu, 레시피, 제조, 디스펜서, 컵 픽업, 재개 consistently.
- Microcopy rules: do not expose internal implementation detail unless it helps operator recovery.

## Implementation constraints
- Framework/styling system: static HTML/CSS/JavaScript served by `voice_screen_node.py`.
- Design-token constraints: no central token system yet; keep colors local and ingredient-specific.
- Performance constraints: catalog rendering should avoid repeated full DOM rebuilds unless catalog data changes.
- Compatibility constraints: ROS nodes publish JSON status; browser UI polls `/api/state`.
- Test/screenshot expectations: run parser/mapper tests for recipe changes and smoke browser/server behavior when launch environment is available.

## Open questions
- [ ] Whether production demos should include real generated drink images per recipe or keep the current deterministic SVG/HTML preview.
- [ ] Whether interrupted pipeline recovery should also surface the checkpoint JSON contents in the operator panel.
