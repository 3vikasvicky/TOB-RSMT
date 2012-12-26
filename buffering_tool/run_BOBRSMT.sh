if (-e "result") then
    rm result
endif

set RC_index = 1
set RT_index = 1
while ($RC_index <= 11)
    roo ../benchmarks/ RC${RC_index}_T_NoObstacle_BOBRSMT 70 >>& result
    @ RC_index = $RC_index + 1   
end
#while ($RT_index <= 5)
#    echo "RT$RT_index" >>& result
#    roo ../benchmarks/ RT$RT_index 0.2 >>& result
#    roo ../benchmarks/ RT$RT_index 0.5 >>& result
#    roo ../benchmarks/ RT$RT_index 0.8 >>& result
#    @ RT_index = $RT_index + 1   
#end
#roo ../benchmarks/ RC1 0.2 >>& result
#roo ../benchmarks/ RC1 0.5 >>& result
#roo ../benchmarks/ RC1 0.8 >>& result
