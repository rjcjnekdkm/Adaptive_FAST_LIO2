# Frozen C generalization summary

The frozen C architecture was evaluated without threshold changes on held-out
GEODE Tunnel4 and Offroad2, using three complete runs of independent FAST-LIO2
A, internal baseline B and C per sequence.

On Tunnel4, C improves over B by 0.76% and is 1.60% worse than A. On Offroad2,
C improves over B by 4.56% and is 2.11% worse than A. C is active and
repeatable in both scenes, with no failed or incomplete trajectory among its
six held-out runs.

The consistent conclusion across structured tunnel and non-structured offroad
is that C improves the internal frontend without regression, but does not
outperform independent FAST-LIO2 under the current aligned sensor
configuration. Claims should retain both baselines and avoid describing these
small GEODE differences as broad superiority.
