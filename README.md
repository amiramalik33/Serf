# Serf

Collection of scripts to design a surface-piercing hydrofoil boat with a v-type rectangular main foil and t-type rear foil.

Roll Stability is a 2DOF simulator (roll angle + height) of just the v-foil
  Running a sweep of this script will give us: v-foil planform (since the plaform is a rectangle, just chord + span), v-angle, and Zcg location
  Running a monte carlo of a few selected configurations will give us the system response and disturbance rejection of each configuration

Pitch Stability is a 2DOF simulator (pitch angle + height) of both foils
  Running a sweep of this script will give us: rear foil moment arm, chord, and span (as it's also rectangular)
  Running a monte carlo of a few selected configurations will give us the system response and disturbance rejection of each configuration

And yes, making these scripts is easier than googling "how to change fluid density in AVL"
