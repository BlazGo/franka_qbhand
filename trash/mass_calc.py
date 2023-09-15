thumb = 0.05 + 0.05 + 0.009
finger = 0.009 + 0.005 + 0.005 + 0.005
palm = 0.266 + 0.266

mass_fingers = thumb + 4* finger
print(f"fingers: {mass_fingers:.3f} [kg]")

mass_hand = palm + mass_fingers
print(f"hand: {mass_hand:.3f} [kg]")

comp_mass = mass_hand*2
print(f"comp_mass: {comp_mass:.3f} [kg]")