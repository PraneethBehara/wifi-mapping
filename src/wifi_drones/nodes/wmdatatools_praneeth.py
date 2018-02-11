
class WifiDataCleaner:

    @classmethod
    def raw2Dict(cls, rawData):
        result = {}
        result2 = {}
        
        for ap in rawData:
            ap_name = ap.addr[0:16]
            if ap_name in result:
                result[ap_name] += int(ap.strength)
                result2[ap_name] += 1
            else:
                result[ap_name] = int(ap.strength)
                result2[ap_name] = 1

        for item in result.keys():
            result[item] = float(result[item])/float(result2[item])
            # print('hello2 %f' % result[item])

        return result