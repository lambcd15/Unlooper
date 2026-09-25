"""Unlooper's processing stages, in the order Unlooper.py runs them:

  gcode_reader      read, clean and unloop the G-code
  toolpath          turn each command into tool movement (segments, distance, time, material)
  motion_planner    acceleration / corner-speed planning (real machine speed and time)
  pixel_coords      time-sampled positions along the planned motion (+ speed / acceleration PNGs)
  scaffold_outputs  the pass that runs the stages above and totals the results
  rendering         move-type PNG and vector SVG preview
  common            shared helpers
"""
