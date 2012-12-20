g++ -w -O2 *.cpp
if (-e "result") then
    rm result
endif

set RC_index = 1
#set RT_index = 1
while ($RC_index <= 12)
#echo "RC$RC_index" >>& result
    ./a.out benchOARST_Timing/RC01_T_NoObstacle.net Table/table.log CapUpperBound.cap > TRUNK_Analysis_Log/execution_RC01_NoObstacle.log
    if (! -d "/home/polaris/yzhang1/rebranching/benchmarks/RC${RC_index}_T_NoObstacle/") then
        mkdir /home/polaris/yzhang1/rebranching/benchmarks/RC${RC_index}_T_NoObstacle
    endif
    cp OARSMT.out /home/polaris/yzhang1/rebranching/benchmarks/RC${RC_index}_T_NoObstacle/RC${RC_index}_T_NoObstacle.branch
#roo . RC$RC_index 0.2 >>& result
#roo . RC$RC_index 0.5 >>& result
#roo . RC$RC_index 0.8 >>& result
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
