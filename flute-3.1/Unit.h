template<typename ANY>
void 
rswap(ANY& obj1, 
            ANY& obj2)
{
      ANY obj = obj1;
        obj1 = obj2;
          obj2 = obj;
}
