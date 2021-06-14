A first example for the "pyGameStructuredStrategyVisualizer.py". Run with:

../../tools/StructuredSlugsParser/compiler.py basicEvasion.structuredslugs > basicEvasion.slugsin
../../tools/pyGameStructuredStrategyVisualizer.py  basicEvasion.png --biasForAction

../../src/slugs --explicitStrategy --jsonOutput basicEvasion.slugsin > test_strategy.json

../../tools/cursesSimulator.py basicEvasion.slugsin




../../../slugs-master/tools/StructuredSlugsParser/compiler.py Belief_Evasion_coarse_seperate_belief_single_obs.structuredslugs > Belief_Evasion_coarse_seperate_belief_single_obs.slugsin
