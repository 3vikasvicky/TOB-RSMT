set RC_index = 1
#set RT_index = 1
while ($RC_index <= 11)
#echo "RC$RC_index" >>& result
    mv RC${RC_index}/flute.branch RC${RC_index}/RC${RC_index}.branch
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