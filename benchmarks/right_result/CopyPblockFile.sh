set RC_index = 11
#set RT_index = 1
while ($RC_index <= 12)
#echo "RC$RC_index" >>& result
    if (! -d RC${RC_index}_BOBRSMT_OUTPUT) then
        mkdir RC${RC_index}_BOBRSMT_OUTPUT
    endif
    cp RC${RC_index}/RC${RC_index}.pblock RC${RC_index}_BOBRSMT_OUTPUT/RC${RC_index}_BOBRSMT_OUTPUT.pblock
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