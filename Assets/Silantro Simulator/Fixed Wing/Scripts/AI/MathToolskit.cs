using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathToolskit
{
    /// <summary>
    /// select m from n
    /// </summary>
    /// <param name="n"></param>
    /// <param name="m"></param>
    /// <returns></returns>
    public static int GetPermutation(int n, int m)
    {
        if(n < m)
        {
            //
            Debug.Log("Permutation error, n must be larger than m");
            return 1;
        }
        return GetFactorial(n) / (GetFactorial(m) * GetFactorial(n - m));
    }

    public static int GetFactorial(int n)
    {
        int currN = n;
        int result = 1;
        while(currN > 1)
        {
            result *= currN;
            currN--;
        }
        return result;
    }
}
