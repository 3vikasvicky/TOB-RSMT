if (-e "result") then
    rm result
endif

set RC_index = 1
set RT_index = 1
while ($RC_index <= 11)
    echo "RC$RC_index" >>& result
    roo . RC${RC_index}_T_NoObstacle 0.2 >>& result
    roo . RC${RC_index}_T_NoObstacle 0.5 >>& result
    roo . RC${RC_index}_T_NoObstacle 0.8 >>& result
    rm /home/polaris/yzhang1/rebranching/benchmarks/RC${RC_index}_BOBRSMT_OUTPUT/RC${RC_index}.branch
    cp output/BOBRSMT.out /home/polaris/yzhang1/rebranching/benchmarks/RC${RC_index}_BOBRSMT_OUTPUT/RC${RC_index}_BOBRSMT_OUTPUT.branch
    @ RC_index = $RC_index + 1   
end
#while ($RT_index <= 5)
#    echo "RT$RT_index" >>& result
#    roo . RT$RT_index 0.2 >>& result
#    roo . RT$RT_index 0.5 >>& result
#    roo . RT$RT_index 0.8 >>& result
#    @ RT_index = $RT_index + 1   
#end
#roo ../benchmarks/ RC1 0.2 >>& result
#roo ../benchmarks/ RC1 0.5 >>& result
#roo ../benchmarks/ RC1 0.8 >>& result
