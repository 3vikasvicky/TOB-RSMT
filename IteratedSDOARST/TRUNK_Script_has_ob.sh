echo Trunk Algorithm Timing Analysis
rm TRUNK_Analysis_Log/*
#echo RC01.net .. Start
#./a.out benchOARST_Timing/RC01_T.net Table/table.log CapUpperBound.cap > TRUNK_Analysis_Log/execution_RC01.log
echo RC1.net .. Start
./a.out RC1_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC1_has_ob.log
echo RC2.net .. Start
./a.out RC2_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC2_has_ob.log
echo RC3.net .. Start
./a.out RC3_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC3_has_ob.log
echo RC4.net .. Start
./a.out RC4_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC4_has_ob.log
echo RC5.net .. Start
./a.out RC5_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC5_has_ob.log
echo RC6.net .. Start
./a.out RC6_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC6_has_ob.log
echo RC7.net .. Start
./a.out RC7_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC7_has_ob.log
echo RC8.net .. Start
./a.out RC8_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC8_has_ob.log
echo RC9.net .. Start
./a.out RC9_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC9_has_ob.log
echo RC10.net .. Start
./a.out RC10_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC10_has_ob.log
echo RC11.net .. Start
./a.out RC11_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC11_has_ob.log
echo RC12.net .. Start
./a.out RC12_T Table/table.log CapUpperBound.cap 70  > TRUNK_Analysis_Log/execution_RC12_has_ob.log
