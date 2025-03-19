def get_input():
    data = []
    print("始点ID, 終点ID, 距離の形式で経路情報を入力してください. (例: 1, 2, 8.54)")
    print("空行を入力する(Enterのみを押す)と経路探索を開始します.")
    
    while True:
        input_path = input("経路情報を入力: ")
        
        # 空行が入力されたら終了
        if input_path == "":
            if len(data) == 0:
                print("経路情報を１つ以上入力してください.")
                continue
            else:
                print("\n探索を開始します.")
                break

        # 入力をカンマで分割し, 余分な空白を削除
        path = [p.strip() for p in input_path.split(",")]
        
        # 入力が3つの値(始点ID, 終点ID, 距離)であることを確認
        if len(path) != 3:
            print("入力形式が正しくありません. 始点ID, 終点ID, 距離をカンマ区切りで入力してください.")
            continue

        try:
            # 始点IDと終点IDを整数、距離を浮動小数点数に変換
            a = int(path[0])
            b = int(path[1])
            c = float(path[2])

            # 駅IDが正の整数であるかを確認
            if a <= 0 or b <= 0:
                print("駅IDは正の整数で入力してください.")
                continue
            # 始点と終点の駅IDが異なるか確認
            if a == b:
                print("始点IDと終点IDは異なるIDを指定してください. (同一駅をループする路線は入力できません.)")
                continue

            data.append((a, b, c))
        
        except ValueError:
            print("数値形式が正しくありません. 始点ID, 終点IDは整数, 距離は浮動小数点数で入力してください.")
            continue

    return data

def normalize_ids(data):
    """点IDを連番に変換する"""
    id_map = {} # 元IDから連番への変換
    reverse_map = {} # 連番から元IDへの変換
    index=0

    for u, v, _ in data:
        if u not in id_map:
            id_map[u] = index
            reverse_map[index] = u
            index += 1
        if v not in id_map:
            id_map[v] = index
            reverse_map[index] = v
            index += 1

    return id_map, reverse_map, len(id_map)
    
def bitDP_max_path(data):
    # ID正規化
    id_map, reverse_map, N = normalize_ids(data)

    # 隣接行列の作成
    adjacency_matrix = [[-1] * N for _ in range(N)]
    for u, v, w in data:
        adjacency_matrix[id_map[u]][id_map[v]] = w  # ノード番号を1始まりに修正
        adjacency_matrix[id_map[v]][id_map[u]] = w  # 無向グラフに対応

    # DP配列と遷移情報を記録する配列の作成
    max_path_dp = [[-1] * N for _ in range(1 << N)] # 最大経路を記録する配列
    path_start  = [[-1] * N for _ in range(1 << N)] # 各経路の始点を記録する配列
    path_prev = [[-1] * N for _ in range(1 << N)] # 終点を記録する配列
    cycle_check = [[-1] * N for _ in range(1 << N)]

    # 初期状態
    for i in range(N):
        max_path_dp[1 << i][i] = 0
        path_start[1 << i][i] = i

    # bit DP
    max_length = 0
    end_state = 0
    last_node = 0

    # 各点から同じ点を通らずに移動できる最大経路の探索
    for i in range(1 << N): # 訪問済みの点の集合を表す状態i(2^N通り)
        for j in range(N): # 任意の点j
            if max_path_dp[i][j] == -1: # 点jを最後に訪問して状態iを満たせない場合
                continue # スキップ
            for k in range(N): # 次に訪問する点k
                if (i >> k) & 1: # 点kを訪問済みの場合
                    continue # スキップ
                if adjacency_matrix[j][k] == -1: # 点jと点k間に経路がない場合
                    continue # スキップ
                new_state = i | (1 << k) # 状態iが点kを通過した新しい状態
                new_length = max_path_dp[i][j] + adjacency_matrix[j][k] # 新しい状態の経路

                if new_length > max_path_dp[new_state][k]: # より長い経路が見つかった場合
                    max_path_dp[new_state][k] = new_length # 最大経路の更新
                    path_prev[new_state][k] = j # 遷移元を記録
                    path_start[new_state][k] = path_start[i][j] # 経路の始点の情報を更新

                    # 最大経路を更新
                    if new_length > max_length:
                        max_length = new_length
                        end_state = new_state
                        last_node = k

    # 探索した経路において終点から始点に移動できる場合の探索
    for i in range(1 << N): # 訪問済みの点の集合を表す状態i(2^N通り)
        for j in range(N): # 任意の点j
            if max_path_dp[i][j] == -1: # 点jを最後に訪問して状態iを満たせない場合
                continue # スキップ
            start_node = path_start[i][j] # 点jを最後に訪問した状態iの始点
            prev_node = path_prev[i][j] if path_prev[i][j] != -1 else None # 点jの1つ前に訪問した点
            if adjacency_matrix[j][start_node] != -1 and prev_node != start_node: # 始点と点j間に経路が存在し，かつ，点jの1つ前に始点を訪問していない場合
                new_length = max_path_dp[i][j] + adjacency_matrix[j][start_node] # 新しい経路長の計算
                if new_length > max_path_dp[i][start_node]: # 始点に再度訪問した方が経路長が長い場合
                    max_path_dp[i][start_node] = new_length # 経路長の更新
                    cycle_check[i][start_node] = 1 # 始点と終点が同じになることを記録
                    if new_length > max_length: # 最大経路より新しい経路が長い場合
                        max_length = new_length # 最大経路の更新
                        end_state = i

    # 経路の復元
    path = [] # 経路を格納するリスト
    state = end_state # 最大経路の状態
    node = last_node # 最大経路の最後の点

    first_point = path_start[state][node] # 最大経路の始点

    # 始点まで遡る
    while node != -1: # 1つ前に訪問した点がなくなるまで
        path.append(reverse_map[node]) # 点IDをリストに格納
        next_node = path_prev[state][node] # 1つ前の点に遡る
        if next_node != -1: # 1つ前の点が存在する
            state ^= 1 << node # 1つ状態を遡る
        node = next_node # 点の更新

    path.reverse()  # 逆順なので反転

    # 最大経路の始点と終点が同じ点の場合, 始点を最後に追加
    if cycle_check[end_state][last_node] == 1 and first_point is not None:
        path.append(reverse_map[first_point])

    return max_length, path

def main():
    data = get_input()
    _, max_path = bitDP_max_path(data) # 探索
    print("\r\n".join(map(str, max_path))) # 結果表示

if __name__ == "__main__":
    main()