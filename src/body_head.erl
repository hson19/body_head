% @doc body_head public API.
-module(body_head).

-behavior(application).

% Callbacks
-export([start/2]).
-export([stop/1,stop_all/0]).
-export([launch/0,launch_all/0]).
-export([launch_e4/0,launch_e4_all/0]).
-export([set_args/1]).

%--- API -----------------------------------------------------------------------
set_args(nav3) ->
    Cn = nav3:calibrate(),
    R0 = e4:calibrate(element(3, Cn)),
    % R0 = e3:calibrate(),
    update_table({{nav3, node()}, Cn}),
    update_table({{e4, node()}, R0}).

launch() ->
    try launch(node_type()) of
        ok ->
            [grisp_led:color(L, green) || L <- [1, 2]],
            ok
    catch
        error:badarg ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            {error, badarg}
    end.
launch_e4() ->
    try launch_e4(node_type()) of
        ok ->
            [grisp_led:color(L, green) || L <- [1, 2]],
            ok
    catch
        error:badarg ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            {error, badarg}
    end.

launch_all() ->
    rpc:multicall(?MODULE, launch, []).
launch_e4_all() ->
    rpc:multicall(?MODULE, launch_e4, []).
stop_all() ->
    _ = rpc:multicall(application, stop, [hera]),
    _ = rpc:multicall(application, start, [hera]),
    ok.
%--- Callbacks -----------------------------------------------------------------

% @private
start(_Type, _Args) -> 
    {ok,Supervisor}=body_head_sup:start_link(),
    init_table(),
    
    io:format("Contents of ETS table args: ~p~n", [ets:tab2list(args)]),
    case node_type() of
        bodypart -> _ = grisp:add_device(spi2,pmod_nav);
        undefined -> 
            _ = net_kernel:set_net_ticktime(8),
            lists:foreach(fun net_kernel:connect_node/1,
            application:get_env(kernel, sync_nodes_optional, []))            
    end,
    _ = launch(),
    {ok,Supervisor}.

% @private
stop(_State) -> 
    io:format("Stopping body_head application~n"),
    ok.



%--- Internal functions ---------------------------------------------------------
launch(bodypart) ->
    Cn = ets:lookup_element(args,{nav3,node()},2), %renvoie le 2 ième élemet de la liste args avec la clé {nav3,node()} ce qui a été mis dans set_argss
    R0 = ets:lookup_element(args, {e4, node()}, 2),
    {ok,_} = hera:start_measure(nav3,Cn),
    {ok,_} = hera:start_measure(e4,R0),
    ok;
launch(_) ->
    ok.
launch_e4(bodypart) ->
    Cn = ets:lookup_element(args,{nav3,node()},2), %renvoie le 2 ième élemet de la liste args avec la clé {nav3,node()} ce qui a été mis dans set_argss
    R0 = ets:lookup_element(args, {e4, node()}, 2),
    {ok,_} = hera:start_measure(nav3,Cn),
    {ok,_} = hera:start_measure(e4,R0),
    ok;
launch_e4(_) ->
    ok.
init_table() ->
    args = ets:new(args, [public, named_table]), % public means any process can read or write the table
    {ResL,_} = rpc:multicall(nodes(), ets, tab2list, [args]),
    L = lists:filter(fun(Res) ->
        case Res of {badrpc,_} -> false; _ -> true end end, ResL),
    lists:foreach(fun(Object) -> ets:insert(args, Object) end, L).


update_table(Object) ->
    _ = rpc:multicall(ets, insert, [args, Object]),
    ok.
node_type() ->
    Host = lists:nthtail(10, atom_to_list(node())), % TODO CHANGE THIS if node name changes
    io:format("Host: ~p~n", [Host]),
    IsBody = lists:prefix("body", Host),
    Ishead = lists:prefix("head", Host),
    Isarm = lists:prefix("arm", Host),
    Isforearm = lists:prefix("forearm", Host),
    io:format("IsBody: ~p~n", [IsBody]),
    io:format("Ishead: ~p~n", [Ishead]),
    io:format("Isarm: ~p~n", [Isarm]),
    io:format("Isforearm: ~p~n", [Isforearm]),
    if
        IsBody -> bodypart;
        Ishead -> bodypart;
        Isarm -> bodypart;
        Isforearm -> bodypart;
        true -> undefined
    end.