if (-e "result") then
    rm result
endif

set RC_index = 1
set RT_index = 1
while ($RC_index <= 12)
    echo "RC$RC_index" >>& result
    roo . RC${RC_index} 70 >>& result
    @ RC_index = $RC_index + 1   
end
while ($RT_index <= 5)
    echo "RT$RT_index" >>& result
    roo . RT$RT_index 70 >>& result
    @ RT_index = $RT_index + 1   
end
