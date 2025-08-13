### README
###### အသုံးပြုပုံ 
- carto မှာ SLAM နှင့် LOCALIZATION ONLY Mode ၂ မျိုးရှိပြီး pure_localization ကို true/false ပေးလို့ရတယ်။
- reeman မှာ localization mode နှင့် slam mode အတွက် .lua file ၂ ခုထားရှိပြီး navigation mode အတွက် carto localization ကိုသာအသုံးပြုသည်။ mapping mode နှင့် remapping mode အတွက် carto slam mode ကိုအသုံးပြုသည်။
- localization တွက် amcl ကိုအသုံးမပြုဘဲ carto localization ကိုသာအသုံးပြုသည်။ 





#### Difference bewteen Mapping and Localization ( Reeman )
```
# mapping.lua
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.mapping_mode = true -- Jcan
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.constraint_builder.min_score = 0.62 -- crease
POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase

# localization.lua
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
POSE_GRAPH.optimize_every_n_nodes = 20
use_pose_extrapolator = true
POSE_GRAPH.mapping_mode = false -- Jcan
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 --0.4
POSE_GRAPH.constraint_builder.min_score = 0.55 -- 0.62
POSE_GRAPH.global_constraint_search_after_n_seconds = 250

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.max_num_final_iterations = 200

POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 40 --10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10.   --4
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.

----TRAJECTORY_BUILDER.collate_fixed_frame = true 

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2)

```


#### Difference bewteen Mapping and Localization ( Chat gpt and Carto research )
```
# mapping.lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

# localization.lua
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1
MAP_BUILDER.num_background_threads = 6
```






### TODO

- Carto ရဲ့ output screen ကို global launch argument နဲ့ အဖွင့် အပိတ် လုပ်ရန်။