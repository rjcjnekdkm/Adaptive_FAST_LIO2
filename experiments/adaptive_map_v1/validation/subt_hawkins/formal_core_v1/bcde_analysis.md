# Formal B/C/D/E core ablation

C and E are the two supported candidates on SubT Hawkins. C provides the lowest
mean and median ATE, establishing positive evidence for basic quality/range
map-insertion filtering. D does not demonstrate an incremental benefit from the
current single-frame normal-direction bin rule.

E restores performance relative to D and has the lowest sample variance and
lowest worst-run ATE of B/C/D/E. Its temporal window enters Persistent mode
repeatedly and changes direction rejection, so this is a behavioral rather than
classification-only result. E nearly matches C's mean while reducing C's sample
standard deviation by 45.61%.

The defensible mechanism statement is: quality/range filtering improves the
baseline on this sequence, while temporal Persistent handling improves
repeatability over the unstable single-frame direction variant. The current
data do not show that direction selection alone improves quality-only C.
